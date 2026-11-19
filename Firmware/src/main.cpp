// Автоматическая лампа в ванной: датчик присутствия HLK-LD2410S + фоторезистор -> лампа 230 В.
// При включении: замкнута перемычка GP7 - GP8 - консоль настройки по USB (console.cpp), иначе сразу работа.
// Кнопка BOOT: удержание SETUP_PRESS_MS - синяя вспышка, отпустили - настройка задержки выключения: синие вспышки -
// задержка в десятках секунд, короткое нажатие +10 с, длинное -10 с, DELAY_EXIT_MS без нажатий - сохранить.
// Калибровка (Readme.md, «Калибровка»): удержание BOOT CAL_PRESS_MS. Без неё (первое включение) - датчик с заводскими
// порогами, порог света LIGHT_DEFAULT_THRESHOLD.
//  0. Индикатор красный / зелёный по очереди, лампа выкл: CAL_LEAVE_S на выход из комнаты.
//  1. Индикатор красный, дверь закрыта, снаружи ходят 1 мин - автокалибровка порогов датчика по фону
//     (команда 0x0009), замер «темно» - в последние CAL_LIGHT_WINDOW_MS шага индикатор погашен (не засвечивает фоторезистор).
//  2. Индикатор зелёный, лампа вкл, внутри стоят и ходят 1 мин - проверка, что датчик видит человека, граница
//     комнаты (дальние ворота), замер «лампа горит» - так же при погашенном индикаторе. Ночник на шаге 2 не горит: свет
//     в ванной включает человек.
//  3. Индикатор не горит - калибровка сохранена. Ошибка - индикатор мигает красным.
// Нет связи с датчиком: не принял настройки за SENSOR_ATTEMPTS попыток - индикатор мигает красным постоянно.
// Обрыв основного стабилитрона D1 (напряжение блока выше SUPPLY_FAULT_MV) - индикатор постоянно часто мигает
// красным, лампа работает как обычно.
// Работа: человек есть и темно - лампа вкл; человека нет (OT2 = 0 после OFF_DELAY_S) - лампа выкл.
// Короткое нажатие BOOT: лампа 230 В не горит - включить, как по датчику (minOnS). Ночник: не горит - дежурная
// подсветка выкл / уровни по кругу (горит, когда темно - свой порог фоторезистора), горит - следующий уровень яркости
// (по кругу); сохраняется во флеш-памяти;
// проценты уровней и порог перегрева - настройка по USB; перегрев по встроенному термодатчику - яркость на 1/3
// меньше не чаще NIGHT_TEMP_STEP_MS, до уровня 0; остыл на NIGHT_TEMP_HYST_C - обратно теми же шагами.
#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "board.h"
#include "config.h"
#include "console.h"
#include "ld2410s.h"

enum class Mode : uint8_t {
    Normal, SetDelay, CalLeave, CalAbsence, CalPresence, Result, Stuck,
#ifdef NIGHT_LIGHT
    SetPwm,   // настройка частоты ШИМ кнопкой
#endif
};

static Calib    s_cal;                        // действующая калибровка
static Mode     s_mode = Mode::Normal;        // режим работы
static bool     s_resultOk = false;           // показ итога калибровки: true - успех, false - ошибка
static uint8_t  s_resultStep = 0;             // показ итога: шаг (цвет / вспышка), выведенный на индикатор
static uint8_t  s_progressStep = 0;           // шаг 1: ход автокалибровки датчика, выведенный в USB (CAL_PROGRESS_STEP)
static uint16_t s_scanReports = 0;            // шаг 1: отчётов датчика любого типа
static uint16_t s_scanProgress = 0;           // шаг 1: последний ход автокалибровки, %
static uint8_t  s_readTries = 0;              // начало шага 2: осталось попыток прочитать пороги
static uint32_t s_readTryMs = 0;              // начало шага 2: время последней попытки (millis)
static bool     s_sensorReady = false;        // шаг 2: пороги прочитаны, датчик в стандартном отчёте - отчёты считаются
static uint32_t s_phaseMs = 0;                // начало шага калибровки, показа ошибки, последнее нажатие в настройке
                                              // задержки и частоты ШИМ (millis)
static uint32_t s_lightStepMs = 0;            // последний замер света на шаге калибровки (millis)
static uint32_t s_lightSum = 0;               // сумма замеров света на шаге калибровки, отсчёты АЦП
static uint16_t s_lightCount = 0;             // число замеров света на шаге калибровки
static bool     s_lightWindow = false;        // конец шага калибровки: индикатор погашен, идут замеры света
static uint16_t s_darkLevel = 0;              // свет на шаге 1 (лампа выкл), отсчёты АЦП
static uint32_t s_trig[SENSOR_GATES];         // пороги срабатывания после автокалибровки шага 1
static uint32_t s_energyMax[SENSOR_GATES];    // шаг 2: максимум энергии по воротам (для журнала)
static uint16_t s_gateHits[SENSOR_GATES];     // шаг 2: кадров с человеком, где энергия ворот >= порога
static uint16_t s_frames = 0;                 // шаг 2: отчётов датчика
static uint16_t s_presentFrames = 0;          // шаг 2: отчётов «человек есть»
static bool     s_lampOn = false;             // лампа включена
static bool     s_lightChecked = false;       // работа: свет уже мерили за это присутствие
static uint32_t s_lightCheckMs = 0;           // работа: время последнего замера света (millis)
static uint32_t s_presentMs = 0;              // работа: последний опрос с «человек есть» на OT2 (millis)
static uint32_t s_stuckSinceMs = 0;           // залипание: начало отсчёта - лампа вкл или свет изменился (millis)
static uint32_t s_stuckLightMs = 0;           // залипание: последний замер света, пока лампа горит (millis)
static uint16_t s_stuckLightRef = 0;          // залипание: свет в начале отсчёта, отсчёты АЦП
static bool     s_stuckRestart = false;       // лампа загорелась: начать отсчёт залипания с now ближайшего прохода
static float    s_stuckBaseDb[SENSOR_GATES];  // режим залипания: фон - максимум энергии ворот за STUCK_BASELINE_MS, дБ
static bool     s_buttonDown = false;         // кнопка нажата (на прошлом опросе)
static uint8_t  s_pressStage = 0;             // нажатие: 0 - короткое, 1 - дольше SETUP_PRESS_MS, 2 - калибровка запущена
static uint32_t s_buttonDownMs = 0;           // начало нажатия (millis)
static bool     s_setupFlashOn = false;       // горит вспышка «прошло SETUP_PRESS_MS»
static bool     s_countFlashOn = false;       // настройка задержки и частоты ШИМ: вспышка счёта сейчас горит
static bool     s_supplyFault = false;        // D1 оборван (держит запасной D2); до выключения питания
static bool     s_sensorFault = false;        // датчик не принял настройки за все попытки (нет связи)
static uint8_t  s_sensorTries = 0;            // осталось попыток записи настроек в датчик
static uint32_t s_sensorTryMs = 0;            // время последней попытки записи настроек в датчик (millis)
#ifndef NIGHT_LIGHT
static uint32_t s_supplyCheckMs = 0;          // время последнего замера напряжения блока (millis)
#else
static uint8_t  s_limitPct = NIGHT_PWM_RANGE; // перегрев: яркость не больше, %; NIGHT_PWM_RANGE - без ограничения
static uint32_t s_tempStepMs = 0;             // время последнего шага яркости по температуре (millis)
static uint32_t s_tempCheckMs = 0;            // время последнего замера температуры (millis)
static bool     s_standbyLit = false;         // дежурная подсветка горит (темно для неё; основной свет - поверх)
static uint32_t s_standbyCheckMs = 0;         // последний замер света для дежурки (millis)
static uint32_t s_standbyQuietMs = 0;         // основной свет или показ уровня ещё были (millis): замер - через
                                              // STANDBY_SETTLE_MS
static bool     s_preview = false;            // светло: новый уровень дежурки показывается NIGHT_PREVIEW_MS
static uint32_t s_previewMs = 0;              // начало показа уровня дежурки (millis)
static int8_t   s_standbyPhase = -1;          // калибровка, шаг 1: уровень дежурки, с которым идёт замер (индекс);
                                              // -1 - замер «темно» или замеров нет
static uint32_t s_standbySum[STANDBY_LEVEL_COUNT];     // суммы замеров света с дежуркой на уровне, отсчёты АЦП
static uint16_t s_standbyCount[STANDBY_LEVEL_COUNT];   // число замеров света с дежуркой на уровне
static uint16_t s_standbyDarkLevel[STANDBY_LEVEL_COUNT];   // свет на шаге 1 с дежуркой на уровне, отсчёты АЦП;
                                                           // 0 - не мерили
#endif

// Индикатор вне калибровки: погашен или мигает - нет связи с датчиком, обрыв D1.
static void ledIdle() {
    boardLed(s_sensorFault ? Led::NoSensor : s_supplyFault ? Led::SupplyFault : Led::Off);
}

#ifndef NIGHT_LIGHT   // у ночника блок питания HLK-5LD05 - стабилитронов нет
static void supplyTick(uint32_t now) {
    if (s_supplyFault || now - s_supplyCheckMs < SUPPLY_CHECK_MS) {
        return;
    }
    s_supplyCheckMs = now;
    const uint16_t mv = boardSupplyMv();
    LOG("блок питания: %u мВ\n", mv);
    if (mv > SUPPLY_FAULT_MV) {
        s_supplyFault = true;
        if (s_mode == Mode::Normal) {
            ledIdle();
        }
    }
}
#endif

// Яркость включённой лампы, %: ночник - выбранный кнопкой уровень, лампа 230 В - полностью.
static uint8_t lampPercent() {
#ifdef NIGHT_LIGHT
    const uint8_t pct = s_cal.nightPct[s_cal.nightLevel];
    return pct < s_limitPct ? pct : s_limitPct;
#else
    return LAMP_FULL_PCT;
#endif
}

// Яркость при погашенном основном свете, %: ночник - дежурная подсветка (горит или показ уровня), лампа 230 В - 0.
static uint8_t offPercent() {
#ifdef NIGHT_LIGHT
    if (s_cal.standbyLevel == 0 || !(s_standbyLit || s_preview)) {
        return 0;
    }
    const uint8_t pct = s_cal.standbyPct[s_cal.standbyLevel - 1];
    return pct < s_limitPct ? pct : s_limitPct;
#else
    return 0;
#endif
}

// Выход лампы по состоянию: основной свет или (ночник) дежурка.
static void applyLamp() {
    boardLamp(s_lampOn ? lampPercent() : offPercent());
}

static void setLamp(bool on) {
    if (on && !s_lampOn) {   // лампа загорелась - отсчёт залипания заново (в stuckCheckTick, по now прохода loop)
        s_stuckRestart = true;
    }
    s_lampOn = on;
    applyLamp();
}

// Свет темнее порога (по направлению кривой фоторезистора).
static bool darker(uint16_t level, uint16_t threshold) {
    return s_cal.ldrInputMode == LDR_DARK_LOW ? level < threshold : level > threshold;
}

static uint16_t clampAdc(int32_t v) {
    return (uint16_t)(v < 0 ? 0 : v > (int32_t)ADC_FULL ? ADC_FULL : v);
}

// Пороги дежурки и основного света при ней - от «темно» dark, размаха «светло - темно» span и добавки света горящей
// дежурки на каждом уровне delta (все - отсчёты АЦП, span и delta - в сторону «светлее»).
static void standbyThresholds(Calib& c, uint16_t dark, uint16_t span, const uint16_t delta[STANDBY_LEVEL_COUNT]) {
    const int32_t dir = c.ldrInputMode == LDR_DARK_HIGH ? -1 : 1;   // «светлее» в отсчётах АЦП: +1 или -1
    const int32_t on = dark + dir * (int32_t)span * STANDBY_ON_PCT / 100;   // дежурка вкл, если темнее
    c.standbyOnThreshold = clampAdc(on);
    for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {
        const int32_t hyst = (int32_t)span * STANDBY_HYST_PCT / 100;   // гистерезис, отсчёты АЦП
        c.standbyOffThreshold[i] = clampAdc(on + dir * ((int32_t)delta[i] + hyst));
        c.lightThresholdStandby[i] = clampAdc(c.lightThreshold + dir * (int32_t)delta[i]);
    }
}

// Пороги дежурки от калибровки света (её нет - середина шкалы как у порога основного света), без добавки
// собственного света (не мерили).
static void standbyFromLight(Calib& c) {
    static const uint16_t NO_DELTA[STANDBY_LEVEL_COUNT] = {};   // свет дежурки не мерили
    memset(c.lightStandbyLevel, 0, sizeof(c.lightStandbyLevel));
    const bool measured = c.lightDarkLevel != c.lightLitLevel;   // свет калиброван
    const uint16_t span = measured ? (c.lightLitLevel > c.lightDarkLevel ? c.lightLitLevel - c.lightDarkLevel
                                                                         : c.lightDarkLevel - c.lightLitLevel)
                                   : clampAdc(2 * (int32_t)c.lightThreshold);
    const uint16_t dark = measured ? c.lightDarkLevel : c.ldrInputMode == LDR_DARK_HIGH ? ADC_FULL : 0;
    standbyThresholds(c, dark, span, NO_DELTA);
}

// Дежурка по умолчанию: выключена, уровни STANDBY_LEVELS, пороги - от калибровки света.
static void standbyDefaults(Calib& c) {
    c.standbyLevel = 0;
    memcpy(c.standbyPct, STANDBY_LEVELS, sizeof(c.standbyPct));
    standbyFromLight(c);
}

void calibDefaults(Calib& c) {
    memset(&c, 0, sizeof(c));
    c.magic = CALIB_MAGIC;
    c.version = CALIB_VERSION;
    c.lightThreshold = LIGHT_DEFAULT_THRESHOLD;
    c.ldrInputMode = LDR_EXISTS ? LDR_DIVIDER_MODE : LDR_NONE;
    c.nightLevel = NIGHT_DEFAULT_LEVEL;   // farGate = 0: датчик не калиброван
    c.offDelay = OFF_DELAY_S;
    memcpy(c.nightPct, NIGHT_LEVELS, sizeof(c.nightPct));
    c.tempMax = NIGHT_TEMP_MAX_C;
    c.minOnS = MIN_ON_DEFAULT_S;
    c.stuckMinutes = STUCK_MINUTES_DEFAULT;   // lightDarkLevel, lightLitLevel = 0: свет не калиброван
    c.stuckLightPct = STUCK_LIGHT_PCT_DEFAULT;
    c.stuckDeltaDb = STUCK_DELTA_DB_DEFAULT;
    c.nightPwmHz = NIGHT_PWM_HZ;
    standbyDefaults(c);
}

// Записывает в датчик сохранённую калибровку (пороги, дальние ворота, задержку) и минимальный отчёт.
// Без калибровки порогов - пороги заводские (не пишутся); farGate = 0 - дальность заводская.
static bool applyCalib() {
    ldBegin(boardSensorBegin());
    const bool ok = ldEnableConfig() &&
                    (!sensorCalibrated(s_cal) || (ldWriteThresholds(LD_CMD_WRITE_TRIG, s_cal.trig) &&
                                             ldWriteThresholds(LD_CMD_WRITE_HOLD, s_cal.hold))) &&
                    ldWriteParams(s_cal.farGate, s_cal.offDelay) && ldSetStandardMode(false);
    ldEndConfig();
    boardSensorEnd();
    LOG("Калибровка записана в датчик: %s\n", ok ? "да" : "ОШИБКА");
    return ok;
}

// Запись настроек в датчик: до SENSOR_ATTEMPTS попыток через SENSOR_RETRY_MS, лампа в это время работает.
static void startSensorWrite() {
    s_sensorTries = SENSOR_ATTEMPTS;
    s_sensorTryMs = millis() - SENSOR_RETRY_MS;   // первая попытка - сразу
}

// Очередная попытка; все неудачны - индикатор мигает постоянно (нет связи с датчиком).
static void sensorWriteTick(uint32_t now) {
    if (s_sensorTries == 0 || now - s_sensorTryMs < SENSOR_RETRY_MS) {
        return;
    }
    s_sensorTryMs = now;
    s_sensorTries--;
    const bool ok = applyCalib();
    if (ok) {
        s_sensorTries = 0;
    }
    if (ok || s_sensorTries == 0) {
        s_sensorFault = !ok;
        ledIdle();
    }
}

// Ход калибровки - в монитор порта USB, если он открыт (USB работает только при LOW_POWER 0 или в отладке).
static void calMsg(const char* fmt, ...) {
#if !LOW_POWER || defined(DEBUG_LOG)
    if (!boardConsoleConnected()) {
        return;
    }
    char text[CAL_MSG_LEN];   // строка сообщения
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    Serial.printf("Калибровка: %s\n", text);
#else
    (void)fmt;
#endif
}

// Показ итога калибровки индикатором: успех - красный / синий / зелёный по CAL_OK_FLASH_MS, ошибка -
// CAL_FAIL_FLASHES вспышек красным по CAL_FAIL_FLASH_MS; затем работа (после ошибки - прежняя калибровка в датчик).
static void startResult(bool ok) {
    s_resultOk = ok;
    s_resultStep = UINT8_MAX;
    s_mode = Mode::Result;
    s_phaseMs = millis();
}

static void resultTick(uint32_t now) {
    static constexpr Led OK_COLORS[] = {Led::Red, Led::Blue, Led::Green};   // успех: цвета по очереди
    const uint32_t flash = s_resultOk ? CAL_OK_FLASH_MS : CAL_FAIL_FLASH_MS;
    const uint8_t steps = s_resultOk ? sizeof(OK_COLORS) : CAL_FAIL_FLASHES * 2;
    const uint32_t step = (now - s_phaseMs) / flash;
    if (step >= steps) {
        ledIdle();
        if (!s_resultOk) {
            startSensorWrite();   // вернуть прежние пороги после неудачной калибровки (без неё - не трогать)
        }
        s_mode = Mode::Normal;
        return;
    }
    if (step != s_resultStep) {
        s_resultStep = (uint8_t)step;
        boardLed(s_resultOk ? OK_COLORS[step] : step % 2 ? Led::Off : Led::Red);
    }
}

// detail - подробность от драйвера датчика (ldLastError) или nullptr.
static void fail(const char* reason, const char* detail = nullptr) {
    LOG("Калибровка: ОШИБКА - %s\n", reason);
    calMsg("ОШИБКА - %s%s%s; LED %u раз красным", reason, detail ? ": " : "", detail ? detail : "",
           CAL_FAIL_FLASHES);
    ldEnableConfig();
    ldSetStandardMode(false);
    ldEndConfig();
    boardSensorEnd();
    setLamp(false);
    startResult(false);
}

static void resetLight(uint32_t now) {
    s_lightSum = 0;
    s_lightCount = 0;
    s_lightWindow = false;
    s_lightStepMs = now - CAL_LIGHT_STEP_MS;   // первый замер - сразу, как начнутся замеры
#ifdef NIGHT_LIGHT
    s_standbyPhase = -1;
    memset(s_standbySum, 0, sizeof(s_standbySum));
    memset(s_standbyCount, 0, sizeof(s_standbyCount));
#endif
}

// Замеры света - только в конце шага длительностью stepMs: индикатор гаснет, чтобы не засвечивать фоторезистор.
// Ночник, шаг 1: после «темно» - по CAL_STANDBY_WINDOW_MS с дежуркой на каждом уровне (её свет на фоторезисторе).
static void sampleLight(uint32_t now, uint32_t stepMs) {
    uint32_t window = CAL_LIGHT_WINDOW_MS;   // длина окна замеров в конце шага, мс
#ifdef NIGHT_LIGHT
    const bool standbyWindow = s_mode == Mode::CalAbsence && s_cal.ldrInputMode != LDR_NONE;   // шаг 1 - и дежурка
    if (standbyWindow) {
        window += STANDBY_LEVEL_COUNT * CAL_STANDBY_WINDOW_MS;
    }
#endif
    const uint32_t end = now - s_phaseMs + window;   // > stepMs - идёт окно замеров
    if (end < stepMs) {
        return;
    }
    uint32_t settled = end - stepMs;   // мс от начала окна (части окна с дежуркой на уровне)
    if (!s_lightWindow) {
        s_lightWindow = true;
        boardLed(Led::Off);
#ifdef NIGHT_LIGHT
        if (s_mode == Mode::CalPresence && s_cal.ldrInputMode != LDR_NONE) {
            setLamp(false);   // замер «светло» - свет ванной, без свечения самого ночника
        }
#endif
    }
#ifdef NIGHT_LIGHT
    if (standbyWindow && settled >= CAL_LIGHT_WINDOW_MS) {
        uint32_t index = (settled - CAL_LIGHT_WINDOW_MS) / CAL_STANDBY_WINDOW_MS;   // уровень дежурки
        index = index < STANDBY_LEVEL_COUNT ? index : STANDBY_LEVEL_COUNT - 1;
        if (s_standbyPhase != (int8_t)index) {
            s_standbyPhase = (int8_t)index;
            boardLamp(s_cal.standbyPct[index]);
        }
        settled = settled - CAL_LIGHT_WINDOW_MS - index * CAL_STANDBY_WINDOW_MS;
    }
#endif
    if (settled >= CAL_LIGHT_SETTLE_MS && now - s_lightStepMs >= CAL_LIGHT_STEP_MS) {
        s_lightStepMs = now;
        const uint16_t level = boardLight();   // отсчёт фоторезистора
#ifdef NIGHT_LIGHT
        if (s_standbyPhase >= 0) {
            s_standbySum[s_standbyPhase] += level;
            s_standbyCount[s_standbyPhase]++;
            return;
        }
#endif
        s_lightSum += level;
        s_lightCount++;
    }
}

// Описание стадий калибровки (консоль calib - перед запуском; кнопка - в монитор порта, если открыт).
void calibPlan(Stream& s) {
    const bool ldr = s_cal.ldrInputMode != LDR_NONE;   // замеры света идут (фоторезистор есть)
    const unsigned long win = CAL_LIGHT_WINDOW_MS / 1000;   // окно замера света в конце шага, с
    s.printf("Калибровка, стадии (всего около %lu с):\n",
             (unsigned long)(CAL_LEAVE_S + CAL_SCAN_S + CAL_SCAN_MARGIN_S + CAL_READ_PAUSE_MS / 1000 + CAL_PRESENCE_S));
    s.printf("  1. ожидание %lu с - выйти из комнаты; LED красный / зелёный\n", (unsigned long)CAL_LEAVE_S);
    s.printf("  2. %lu с снаружи, двери закрыты, в зоне датчика никого - автокалибровка датчика по фону; "
             "LED красный\n", (unsigned long)(CAL_SCAN_S + CAL_SCAN_MARGIN_S));
    if (ldr) {
#ifdef NIGHT_LIGHT
        s.printf("     последние %lu с LED не горит: %lu с замер «темно», затем по %lu с ночник светит уровнями "
                 "дежурной подсветки", win + STANDBY_LEVEL_COUNT * CAL_STANDBY_WINDOW_MS / 1000, win,
                 (unsigned long)(CAL_STANDBY_WINDOW_MS / 1000));
        for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {
            s.printf(" %u", s_cal.standbyPct[i]);
        }
        s.printf(" %% - замер её света\n");
#else
        s.printf("     последние %lu с LED не горит - замер «темно»\n", win);
#endif
    }
    s.printf("     затем пауза %lu с и чтение порогов из датчика (до %u попыток раз в %lu с) - оставаться снаружи; "
             "LED красный\n", (unsigned long)(CAL_READ_PAUSE_MS / 1000), CAL_READ_TRIES,
             (unsigned long)(CAL_READ_RETRY_MS / 1000));
    s.printf("  3. %lu с войти и ходить внутри - свет загорается; LED зелёный\n", (unsigned long)CAL_PRESENCE_S);
    if (ldr) {
#ifdef NIGHT_LIGHT
        s.printf("     включить свет в ванной; последние %lu с LED и ночник не горят - замер «светло»\n", win);
#else
        s.printf("     последние %lu с LED не горит - замер «лампа горит»\n", win);
#endif
    }
    s.printf("  4. итог: успех - LED красный / синий / зелёный, ошибка - %u вспышек красным (причина - в мониторе)\n",
             CAL_FAIL_FLASHES);
}

// Начало калибровки: лампа выкл, индикатор красный / зелёный по очереди - CAL_LEAVE_S на выход из комнаты.
// showPlan - вывести стадии в монитор порта (запуск кнопкой; из консоли они уже показаны).
// now - время прохода loop, не millis(): после вывода стадий оно больше now, и CalLeave того же прохода
// (now - s_phaseMs через 0) сразу считал бы паузу прошедшей - шаг 1 без паузы на выход.
static void startCalibration(bool showPlan, uint32_t now) {
    LOG("Калибровка: %lu с выйти из комнаты\n", (unsigned long)CAL_LEAVE_S);
#if !LOW_POWER || defined(DEBUG_LOG)
    if (showPlan && boardConsoleConnected()) {
        calibPlan(Serial);
    }
#else
    (void)showPlan;
#endif
    calMsg("1. ожидание %lu с - выйти из комнаты; LED красный / зелёный", (unsigned long)CAL_LEAVE_S);
    if (s_mode == Mode::Stuck) {
        boardSensorEnd();   // режим залипания кончился; калибровка откроет UART сама (датчик - в режим команд)
    }
#ifdef NIGHT_LIGHT
    s_standbyLit = false;   // дежурка в калибровке не горит (кроме замера с ней в шаге 1)
    s_preview = false;
#endif
    setLamp(false);
    boardLed(Led::CalLeave);
    s_mode = Mode::CalLeave;
    s_phaseMs = now;
}

// Шаг 1: индикатор красный, автокалибровка порогов датчика по фону.
static void startAbsence(uint32_t now) {
    LOG("Калибровка, шаг 1: отсутствие\n");
    boardLed(Led::Red);
    ldBegin(boardSensorBegin());
    const bool ok = ldEnableConfig() && ldStartAutoThreshold(AUTO_TRIG_FACTOR, AUTO_HOLD_FACTOR, CAL_SCAN_S);
    ldEndConfig();
    LOG("Автокалибровка датчика запущена: %s\n", ok ? "да" : "нет ответа");   // ACK 0x0009 в протоколе неясен
    calMsg("2. снаружи, двери закрыты, %lu с (автокалибровка датчика по фону: %s); LED красный, последние %lu с "
           "не горит", (unsigned long)(CAL_SCAN_S + CAL_SCAN_MARGIN_S), ok ? "запущена" : "нет ответа",
#ifdef NIGHT_LIGHT
           (unsigned long)((CAL_LIGHT_WINDOW_MS +
                            (s_cal.ldrInputMode != LDR_NONE ? STANDBY_LEVEL_COUNT * CAL_STANDBY_WINDOW_MS : 0)) / 1000));
#else
           (unsigned long)(CAL_LIGHT_WINDOW_MS / 1000));
#endif
    (void)ok;
    resetLight(now);
    s_progressStep = UINT8_MAX;
    s_scanReports = 0;
    s_scanProgress = 0;
    s_mode = Mode::CalAbsence;
    s_phaseMs = now;
}

// Конец шага 1: пауза CAL_READ_PAUSE_MS, затем readScanThresholds - после автокалибровки датчик несколько секунд
// не отвечает и сбивается (work/ld2410s_sniff_notes.md). Лампа выкл, индикатор красный - снаружи.
static void startPresence(uint32_t now) {
    calMsg("за шаг отчётов датчика %u, последний ход автокалибровки %u %%", s_scanReports, s_scanProgress);
    if (s_lightCount == 0) {
        fail("нет замеров света");
        return;
    }
    s_darkLevel = (uint16_t)(s_lightSum / s_lightCount);
#ifdef NIGHT_LIGHT
    setLamp(false);   // конец замера с дежуркой
    for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {
        s_standbyDarkLevel[i] = s_standbyCount[i] ? (uint16_t)(s_standbySum[i] / s_standbyCount[i]) : 0;
        if (s_standbyCount[i]) {
            calMsg("свет: темно %u, с дежурной подсветкой %u %% - %u", s_darkLevel, s_cal.standbyPct[i],
                   s_standbyDarkLevel[i]);
        }
    }
#endif
    calMsg("пауза %lu с, затем чтение порогов из датчика (до %u попыток раз в %lu с); LED красный - оставаться снаружи",
           (unsigned long)(CAL_READ_PAUSE_MS / 1000), CAL_READ_TRIES, (unsigned long)(CAL_READ_RETRY_MS / 1000));
    boardLed(Led::Red);
    s_readTries = CAL_READ_TRIES;
    s_readTryMs = now;
    s_sensorReady = false;
    s_mode = Mode::CalPresence;
}

// Шаг 2 после паузы и при повторах: пороги шага 1 из датчика, стандартный отчёт (энергия по воротам);
// удалось - лампа вкл, индикатор зелёный, отсчёт CAL_PRESENCE_S.
static void readScanThresholds(uint32_t now) {
    const char* err = !ldEnableConfig()                          ? "датчик не отвечает (вход в режим команд)"
                      : !ldReadThresholds(LD_CMD_READ_TRIG, s_trig) ? "датчик не отвечает (чтение порогов)"
                      : !ldSetStandardMode(true)                     ? "датчик не отвечает (стандартный отчёт)"
                                                                     : nullptr;   // причина ошибки или nullptr
    char detail[CAL_MSG_LEN];   // подробность от драйвера - копия до ldEndConfig, он её перезапишет
    snprintf(detail, sizeof(detail), "%s", ldLastError());
    ldEndConfig();
    if (err && --s_readTries) {   // повтор через CAL_READ_RETRY_MS (CalPresence в loop)
        calMsg("%s: %s; повтор, осталось попыток %u", err, detail, s_readTries);
        return;
    }
    if (err) {
        fail(err, detail);
        return;
    }
    s_sensorReady = true;
    LOG("Калибровка, шаг 2: присутствие (темно = %u)\n", s_darkLevel);
    calMsg("пороги шага 1 прочитаны, датчик в стандартном отчёте");
    calMsg("3. войти и ходить внутри %lu с%s; LED зелёный, последние %lu с не горит", (unsigned long)CAL_PRESENCE_S,
#ifdef NIGHT_LIGHT
           s_cal.ldrInputMode == LDR_NONE ? "" : ", включить свет в ванной",
#else
           "",
#endif
           (unsigned long)(CAL_LIGHT_WINDOW_MS / 1000));
    setLamp(true);   // свет шага 2 - сигнал «войти»; лампа 230 В - и замер «лампа горит»
    boardLed(Led::Green);
    memset(s_energyMax, 0, sizeof(s_energyMax));
    memset(s_gateHits, 0, sizeof(s_gateHits));
    s_frames = 0;
    s_presentFrames = 0;
    resetLight(now);
    s_phaseMs = now;
}

static void countReport(const LdReport& r) {
    if (r.type > 1) {
        return;
    }
    s_frames++;
    if (!r.present) {
        return;
    }
    s_presentFrames++;
    if (r.type != 1) {
        return;
    }
    // Энергия в отчёте - линейная, пороги 0x0072/0x0073 - дБ: программа HLK рисует энергию как 10 * log10(E)
    // (перехват work/uart_sniff_20261111_102346.log совпал с её графиком; в протоколе не сказано)
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        if (r.energy[g] > s_energyMax[g]) {
            s_energyMax[g] = r.energy[g];
        }
        if (r.energy[g] > 0 && ENERGY_DB_SCALE * log10f((float)r.energy[g]) >= (float)s_trig[g]) {
            s_gateHits[g]++;
        }
    }
}

static void finishCalibration() {
    const uint16_t lit = s_lightCount ? (uint16_t)(s_lightSum / s_lightCount) : s_darkLevel;
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        LOG("ворота %2u: порог %lu, энергия макс %lu, попаданий %u\n", g, (unsigned long)s_trig[g],
            (unsigned long)s_energyMax[g], s_gateHits[g]);
    }
    LOG("кадров %u, с человеком %u; свет: темно %u, лампа %u\n", s_frames, s_presentFrames, s_darkLevel, lit);
    calMsg("отчётов датчика %u (нужно >= %u), с человеком %u (нужно >= %u %%); свет: темно %u, светло %u",
           s_frames, CAL_MIN_FRAMES, s_presentFrames, CAL_PRESENT_PCT, s_darkLevel, lit);

    if (s_frames < CAL_MIN_FRAMES) {
        fail("нет отчётов датчика");
        return;
    }
    if ((uint32_t)s_presentFrames * 100 < (uint32_t)CAL_PRESENT_PCT * s_frames) {
        fail("датчик плохо видит человека");
        return;
    }
    const bool noLdr = s_cal.ldrInputMode == LDR_NONE;   // работа без фоторезистора - свет не проверять
    const uint16_t diff = lit > s_darkLevel ? lit - s_darkLevel : s_darkLevel - lit;
    if (!noLdr && diff < LIGHT_MIN_DIFF) {
        fail("фоторезистор не видит лампу");
        return;
    }

    // Граница комнаты: самые дальние ворота, где энергия человека выше порога в >= CAL_GATE_HIT_PCT кадров.
    // Параметр «дальние ворота» 1..16 (табл. 2-2), ворота 0..15: g + 1 - не меньше ворот g.
    uint8_t farGate = SENSOR_FAR_MAX;
    for (int8_t g = SENSOR_GATES - 1; g >= 0; g--) {
        if ((uint32_t)s_gateHits[g] * 100 >= (uint32_t)CAL_GATE_HIT_PCT * s_presentFrames) {
            farGate = (uint8_t)(g + 1);
            break;
        }
    }
    LOG("дальние ворота: %u\n", farGate);

    Calib c;
    c.magic = CALIB_MAGIC;
    c.version = CALIB_VERSION;
    memcpy(c.trig, s_trig, sizeof(c.trig));
    c.farGate = farGate;
    c.lightThreshold = noLdr ? s_cal.lightThreshold : (uint16_t)((lit + s_darkLevel) / 2);
    c.ldrInputMode = noLdr ? LDR_NONE : lit > s_darkLevel ? LDR_DARK_LOW : LDR_DARK_HIGH;
    c.nightLevel = s_cal.nightLevel;   // перекалибровка уровни, порог перегрева и задержку не сбрасывает
    c.offDelay = s_cal.offDelay;
    memcpy(c.nightPct, s_cal.nightPct, sizeof(c.nightPct));
    c.tempMax = s_cal.tempMax;
    c.minOnS = s_cal.minOnS;
    c.lightDarkLevel = noLdr ? 0 : s_darkLevel;   // для проверки залипания: свет «не меняется»
    c.lightLitLevel = noLdr ? 0 : lit;
    c.stuckMinutes = s_cal.stuckMinutes;
    c.stuckLightPct = s_cal.stuckLightPct;
    c.stuckDeltaDb = s_cal.stuckDeltaDb;
    c.nightPwmHz = s_cal.nightPwmHz;
    standbyDefaults(c);
#ifdef NIGHT_LIGHT
    c.standbyLevel = s_cal.standbyLevel;   // выбор кнопкой и проценты уровней калибровка не сбрасывает
    memcpy(c.standbyPct, s_cal.standbyPct, sizeof(c.standbyPct));
    if (!noLdr && s_standbyDarkLevel[0]) {   // замеры с дежуркой были (все уровни подряд)
        const bool rising = c.ldrInputMode == LDR_DARK_LOW;   // больше света - больше отсчёт
        uint16_t delta[STANDBY_LEVEL_COUNT];                  // свет дежурки на фоторезисторе по уровням
        for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {
            c.lightStandbyLevel[i] = s_standbyDarkLevel[i];
            const int32_t add = (rising ? 1 : -1) * ((int32_t)s_standbyDarkLevel[i] - s_darkLevel);
            delta[i] = (uint16_t)(add > 0 ? add : 0);
        }
        standbyThresholds(c, s_darkLevel, diff, delta);
        calMsg("дежурка: вкл темнее %u", c.standbyOnThreshold);
        for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {
            calMsg("дежурка %u %%: свет с ней %u, погасить светлее %u; основной свет при ней - темнее %u",
                   c.standbyPct[i], c.lightStandbyLevel[i], c.standbyOffThreshold[i], c.lightThresholdStandby[i]);
            if (rising ? c.standbyOffThreshold[i] >= lit : c.standbyOffThreshold[i] <= lit) {
                calMsg("ВНИМАНИЕ: свет дежурки %u %% на фоторезисторе больше разницы «светло - темно» - при включённом "
                       "свете в ванной она может не гаснуть (поправить sboff)", c.standbyPct[i]);
            }
        }
    }
#endif
    const bool ok = ldEnableConfig() && ldReadThresholds(LD_CMD_READ_HOLD, c.hold) &&
                    ldWriteParams(farGate, c.offDelay) && ldSetStandardMode(false);
    ldEndConfig();
    if (!ok) {
        fail("датчик не отвечает");
        return;
    }
    boardSensorEnd();
    s_cal = c;
    boardSave(s_cal);
    LOG("Калибровка сохранена\n");
    calMsg("4. завершена успешно, дальние ворота %u; LED красный / синий / зелёный", farGate);
    s_sensorFault = false;   // датчик только что ответил
    s_sensorTries = 0;
    s_lightChecked = true;   // человек внутри, лампа горит - дальше по OT2
    startResult(true);
}

// Темно ли (по фоторезистору и порогу калибровки; ночник при горящей дежурке - её порог: фоторезистор видит её свет);
// без фоторезистора - всегда темно.
static bool isDark() {
    if (s_cal.ldrInputMode == LDR_NONE) {
        return true;
    }
#ifdef NIGHT_LIGHT
    const uint16_t threshold = s_standbyLit && s_cal.standbyLevel   // порог «темно»
                                   ? s_cal.lightThresholdStandby[s_cal.standbyLevel - 1] : s_cal.lightThreshold;
#else
    const uint16_t threshold = s_cal.lightThreshold;
#endif
    const uint16_t level = boardLight();
    const bool dark = darker(level, threshold);
    LOG("человек, свет %u (порог %u): %s\n", level, threshold, dark ? "темно - лампа вкл" : "светло");
    return dark;
}

static void startStuck(uint32_t now);

static void normalTick(uint32_t now) {
    const bool present = boardPresence();
    if (present) {
        s_presentMs = now;   // каждый импульс OT2 продлевает свет на minOnS
    }
    if (s_lampOn) {
        if (!present && now - s_presentMs >= (uint32_t)s_cal.minOnS * 1000) {
            setLamp(false);
            s_lightChecked = false;
        }
        return;
    }
    if (!present) {
        s_lightChecked = false;
        return;
    }
    if (s_lightChecked && now - s_lightCheckMs < LIGHT_RECHECK_MS) {
        return;
    }
    s_lightChecked = true;
    s_lightCheckMs = now;
    if (isDark()) {
        setLamp(true);
    }
}

// Залипание: лампа горит s_cal.stuckMinutes без перерыва, свет (если есть калибровка света) не выходит из
// полосы ± stuckLightPct % от «светло - темно» - предмет у датчика держит «человек есть». Свет изменился - отсчёт заново.
static void stuckCheckTick(uint32_t now) {
    if (!s_lampOn || s_cal.stuckMinutes == 0) {
        return;
    }
    if (s_stuckRestart) {   // не millis() в setLamp: оно позже now, now - s_stuckSinceMs ушло бы через 0
        s_stuckRestart = false;
        s_stuckSinceMs = s_stuckLightMs = now;
        s_stuckLightRef = boardLight();
    }
    const uint16_t span = s_cal.lightLitLevel > s_cal.lightDarkLevel ? s_cal.lightLitLevel - s_cal.lightDarkLevel
                                                                     : s_cal.lightDarkLevel - s_cal.lightLitLevel;
    const uint16_t band = (uint16_t)((uint32_t)span * s_cal.stuckLightPct / 100);   // полоса «не меняется», отсчёты
    if (s_cal.ldrInputMode != LDR_NONE && band > 0 && now - s_stuckLightMs >= STUCK_LIGHT_CHECK_MS) {
        s_stuckLightMs = now;
        const uint16_t level = boardLight();
        if (level > s_stuckLightRef + band || level + band < s_stuckLightRef) {
            s_stuckSinceMs = now;
            s_stuckLightRef = level;
        }
    }
    if (now - s_stuckSinceMs >= (uint32_t)s_cal.stuckMinutes * 60000UL) {
        startStuck(now);
    }
}

// Режим залипания: лампа выкл, индикатор - белые вспышки, датчик - в стандартный отчёт (энергия по воротам).
static void startStuck(uint32_t now) {
    LOG("Залипание: лампа %u мин без перерыва - режим по энергии ворот\n", s_cal.stuckMinutes);
    setLamp(false);
    boardLed(Led::StuckFlash);
    ldBegin(boardSensorBegin());
    const bool ok = ldEnableConfig() && ldSetStandardMode(true);
    ldEndConfig();
    LOG("стандартный отчёт: %s\n", ok ? "да" : "нет ответа");   // нет - лампа не включится, выход по OT2 = 0
    (void)ok;
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        s_stuckBaseDb[g] = 0;
    }
    s_lightChecked = false;
    s_phaseMs = now;
    s_mode = Mode::Stuck;
}

// Выход из залипания: датчик в минимальный отчёт, UART выкл, обычная работа (лампа - по OT2 и minOnS).
static void finishStuck() {
    LOG("Залипание кончилось: OT2 = 0\n");
    ldEnableConfig();
    ldSetStandardMode(false);
    ldEndConfig();
    boardSensorEnd();
    s_mode = Mode::Normal;
    ledIdle();
}

// Режим залипания: фон - первые STUCK_BASELINE_MS; потом любые ворота выше фона на stuckDeltaDb - «человек»:
// лампа вкл (если темно), гаснет через minOnS после последнего превышения.
static void stuckTick(uint32_t now) {
    if (!boardPresence()) {
        finishStuck();
        return;
    }
    const bool baseline = now - s_phaseMs < STUCK_BASELINE_MS;   // идёт замер фона
    bool above = false;   // в принятых отчётах есть ворота выше фона
    LdReport r;
    while (ldPoll(r)) {
        if (r.type != 1) {
            continue;
        }
        for (uint8_t g = 0; g < SENSOR_GATES; g++) {
            const float db = r.energy[g] > 0 ? ENERGY_DB_SCALE * log10f((float)r.energy[g]) : 0.0f;   // дБ, как пороги
            if (baseline) {
                s_stuckBaseDb[g] = db > s_stuckBaseDb[g] ? db : s_stuckBaseDb[g];
            } else if (db >= s_stuckBaseDb[g] + s_cal.stuckDeltaDb) {
                above = true;
            }
        }
    }
    if (above) {
        s_presentMs = now;
        if (!s_lampOn && isDark()) {
            setLamp(true);
        }
    } else if (s_lampOn && now - s_presentMs >= (uint32_t)s_cal.minOnS * 1000) {
        setLamp(false);
    }
}

#ifdef NIGHT_LIGHT
// Температура раз в NIGHT_TEMP_CHECK_MS (и при погашенном ночнике - чтобы остывание вернуло яркость). Шаг яркости -
// не чаще NIGHT_TEMP_STEP_MS: горит и выше tmax - на 1/3 меньше, не ниже уровня 0; ниже tmax - NIGHT_TEMP_HYST_C -
// обратно x3/2, дошли до выбранного уровня - без ограничения. Между порогами (гистерезис) - без изменений.
static void tempTick(uint32_t now) {
    if (now - s_tempCheckMs < NIGHT_TEMP_CHECK_MS) {
        return;
    }
    s_tempCheckMs = now;
    const int16_t t = boardTempC() + NIGHT_TEMP_OFFSET_C;
    LOG("температура %d °C, предел яркости %u %%\n", t, s_limitPct);
    if (now - s_tempStepMs < NIGHT_TEMP_STEP_MS) {
        return;
    }
    const uint8_t level = s_cal.nightPct[s_cal.nightLevel];
    uint16_t limit = s_limitPct;
    if (s_lampOn && t > s_cal.tempMax) {
        limit = (level < s_limitPct ? level : s_limitPct) * NIGHT_DERATE_NUM / NIGHT_DERATE_DEN;
        if (limit < s_cal.nightPct[0]) {
            limit = s_cal.nightPct[0];
        }
    } else if (t < s_cal.tempMax - NIGHT_TEMP_HYST_C && s_limitPct < NIGHT_PWM_RANGE) {
        limit = s_limitPct * NIGHT_DERATE_DEN / NIGHT_DERATE_NUM;
        if (limit >= level) {
            limit = NIGHT_PWM_RANGE;
        }
    }
    if (limit == s_limitPct) {
        return;
    }
    s_limitPct = (uint8_t)limit;
    s_tempStepMs = now;
    LOG("перегрев: предел яркости %u %%\n", s_limitPct);
    applyLamp();   // новая яркость - основному свету или дежурке
}

// Дежурка должна гореть: уровень выбран и темно - пока не горит, темнее порога вкл; горит (фоторезистор видит её
// свет) - пока не светлее порога «погасить». Без фоторезистора - всегда темно.
static bool standbyShouldLight() {
    if (s_cal.standbyLevel == 0) {
        return false;
    }
    if (s_cal.ldrInputMode == LDR_NONE) {
        return true;
    }
    return darker(boardLight(),
                  s_standbyLit ? s_cal.standbyOffThreshold[s_cal.standbyLevel - 1] : s_cal.standbyOnThreshold);
}

// Дежурка: конец показа уровня; замер света раз в STANDBY_CHECK_MS, пока основной свет не горит (после него и после
// показа уровня - не раньше STANDBY_SETTLE_MS: фоторезистор ещё видит их свет).
static void standbyTick(uint32_t now) {
    if (s_preview && now - s_previewMs >= NIGHT_PREVIEW_MS) {
        s_preview = false;
        applyLamp();
    }
    if (s_lampOn || s_preview) {
        s_standbyQuietMs = now;
        return;
    }
    if (now - s_standbyQuietMs < STANDBY_SETTLE_MS || now - s_standbyCheckMs < STANDBY_CHECK_MS) {
        return;
    }
    s_standbyCheckMs = now;
    const bool lit = standbyShouldLight();
    if (lit != s_standbyLit) {
        s_standbyLit = lit;
        LOG("дежурка: %s\n", lit ? "темно - горит" : "светло - погашена");
        applyLamp();
    }
}

// Короткое нажатие при погашенном основном свете: дежурка выкл / уровень 1 / 2 / 3 по кругу, сохранить; темно - горит
// сразу на новом уровне, светло - уровень показывается NIGHT_PREVIEW_MS.
static void nextStandby(uint32_t now) {
    s_cal.standbyLevel = (uint8_t)((s_cal.standbyLevel + 1) % (STANDBY_LEVEL_COUNT + 1));
    boardSave(s_cal);
    s_standbyLit = standbyShouldLight();
    s_preview = s_cal.standbyLevel != 0 && !s_standbyLit;
    s_previewMs = now;
    s_standbyCheckMs = now;
    LOG("дежурка: уровень %u (%u %%)\n", s_cal.standbyLevel,
        s_cal.standbyLevel ? s_cal.standbyPct[s_cal.standbyLevel - 1] : 0);
    applyLamp();
}
#endif

// Короткое нажатие (пользователь). Лампа 230 В: включить, как по датчику (свет не проверяется - включили рукой),
// гаснет через minOnS без присутствия. Ночник: основной свет не горит - дежурная подсветка по кругу, горит - следующий
// уровень яркости по кругу, сохранить.
static void shortPress(uint32_t now) {
    s_presentMs = now;   // как импульс OT2: горит не меньше minOnS от нажатия
#ifdef NIGHT_LIGHT
    if (!s_lampOn) {
        nextStandby(now);
        return;
    }
    s_cal.nightLevel = (uint8_t)((s_cal.nightLevel + 1) % NIGHT_LEVEL_COUNT);
    boardSave(s_cal);
#endif
    LOG("кнопка: лампа %u %%\n", lampPercent());
    setLamp(true);
}

// Настройка задержки и частоты ШИМ: показ счёта заново - с последнего нажатия.
static void startCountShow(uint32_t now) {
    s_phaseMs = now;
    s_countFlashOn = false;
    boardLed(Led::Off);
}

static void startSetDelay(uint32_t now) {
    LOG("Настройка задержки: %u с\n", s_cal.offDelay);
    s_mode = Mode::SetDelay;
    startCountShow(now);
}

// Шаг задержки: +1 / -1 шаг OFF_DELAY_STEP_S в пределах OFF_DELAY_MIN_S..OFF_DELAY_MAX_S.
static void stepDelay(int8_t dir, uint32_t now) {
    const int16_t v = s_cal.offDelay + dir * OFF_DELAY_STEP_S;
    if (v >= OFF_DELAY_MIN_S && v <= OFF_DELAY_MAX_S) {
        s_cal.offDelay = (uint8_t)v;
    }
    LOG("задержка: %u с\n", s_cal.offDelay);
    startCountShow(now);
}

// Счёт: count вспышек цвета color, пауза FLASH_PAUSE_MS, по кругу (задержка - синим, частота ШИМ - зелёным).
static void countShowTick(uint32_t now, uint32_t count, Led color) {
    const uint32_t period = FLASH_ON_MS + FLASH_OFF_MS;
    const uint32_t t = (now - s_phaseMs) % (count * period + FLASH_PAUSE_MS);
    const bool on = t < count * period && t % period < FLASH_ON_MS;
    if (on != s_countFlashOn) {
        s_countFlashOn = on;
        boardLed(on ? color : Led::Off);
    }
}

// Выход из настройки задержки: сохранить, записать в датчик.
static void finishSetDelay() {
    boardSave(s_cal);
    LOG("Задержка сохранена: %u с\n", s_cal.offDelay);
    s_mode = Mode::Normal;
    ledIdle();
    startSensorWrite();
}

#ifdef NIGHT_LIGHT
// Частота ШИМ khz кГц: в настройку и в ШИМ, ночник горит на выбранном уровне.
static void applyPwmKhz(uint32_t khz) {
    s_cal.nightPwmHz = khz * HZ_PER_KHZ;
    LOG("частота ШИМ: %lu Гц\n", (unsigned long)s_cal.nightPwmHz);
    boardLampFreq(s_cal.nightPwmHz);   // гасит ночник - зажечь снова
    setLamp(true);
}

// Из настройки задержки (удержание): задержку сохранить, частота - целые кГц в пределах настройки, зелёный - пока держат.
static void startSetPwm(uint32_t now) {
    finishSetDelay();   // запись задержки в датчик - после выхода в обычную работу
    uint32_t khz = s_cal.nightPwmHz / HZ_PER_KHZ;   // целые кГц (3500 Гц -> 3)
    khz = khz < PWM_SETUP_KHZ_MIN ? PWM_SETUP_KHZ_MIN : khz > PWM_SETUP_KHZ_MAX ? PWM_SETUP_KHZ_MAX : khz;
    s_mode = Mode::SetPwm;
    s_phaseMs = now;
    boardLed(Led::Green);
    applyPwmKhz(khz);
}

// Шаг частоты: +1 / -1 кГц по кругу PWM_SETUP_KHZ_MIN..PWM_SETUP_KHZ_MAX.
static void stepPwm(int8_t dir, uint32_t now) {
    uint32_t khz = s_cal.nightPwmHz / HZ_PER_KHZ + dir;
    khz = khz > PWM_SETUP_KHZ_MAX ? PWM_SETUP_KHZ_MIN : khz < PWM_SETUP_KHZ_MIN ? PWM_SETUP_KHZ_MAX : khz;
    applyPwmKhz(khz);
    startCountShow(now);
}

// Выход из настройки частоты: сохранить, ночник погасить - дальше по присутствию и свету.
static void finishSetPwm() {
    boardSave(s_cal);
    LOG("Частота ШИМ сохранена: %lu Гц\n", (unsigned long)s_cal.nightPwmHz);
    s_mode = Mode::Normal;
    ledIdle();
    setLamp(false);
    s_lightChecked = false;
}
#endif

// Удержание CAL_PRESS_MS: в настройке задержки ночника - настройка частоты ШИМ, иначе - калибровка.
static void longHold(uint32_t now) {
#ifdef NIGHT_LIGHT
    if (s_mode == Mode::SetDelay) {
        startSetPwm(now);
        return;
    }
#endif
    startCalibration(true, now);
}

// Кнопка: удержание CAL_PRESS_MS - калибровка (в настройке задержки ночника - настройка частоты ШИМ); SETUP_PRESS_MS
// и отпустили - настройка задержки; короткое - включить (ночник горит - яркость). В настройке задержки и частоты: короче
// DELAY_DEC_PRESS_MS - плюс шаг, дольше - минус.
static void buttonTick(uint32_t now) {
    if (s_setupFlashOn && now - s_buttonDownMs >= SETUP_PRESS_MS + FLASH_ON_MS) {
        s_setupFlashOn = false;
        if (s_mode == Mode::Normal) {
            ledIdle();
        }
    }
    if (boardButton()) {
        if (!s_buttonDown) {
            s_buttonDown = true;
            s_pressStage = 0;
            s_buttonDownMs = now;
        }
        const uint32_t held = now - s_buttonDownMs;
        if (s_pressStage < 2 && held >= CAL_PRESS_MS) {
            s_pressStage = 2;
            longHold(now);
        } else if (s_pressStage == 0 && held >= SETUP_PRESS_MS && s_mode == Mode::Normal) {
            s_pressStage = 1;
            s_setupFlashOn = true;
            boardLed(Led::Blue);
        }
        return;
    }
    if (!s_buttonDown) {
        return;
    }
    s_buttonDown = false;
    const int8_t dir = now - s_buttonDownMs < DELAY_DEC_PRESS_MS ? 1 : -1;   // настройка: короткое +, длинное -
    if (s_pressStage == 2) {
#ifdef NIGHT_LIGHT
        if (s_mode == Mode::SetPwm) {
            startCountShow(now);   // отпустили после входа: зелёный гаснет, счёт кГц
        }
#endif
        return;
    }
    if (s_mode == Mode::SetDelay) {
        stepDelay(dir, now);
#ifdef NIGHT_LIGHT
    } else if (s_mode == Mode::SetPwm) {
        stepPwm(dir, now);
#endif
    } else if (s_mode == Mode::Normal && s_pressStage == 1) {
        startSetDelay(now);
    } else if (s_mode == Mode::Normal) {
        shortPress(now);
    }
}

// Калибровка из флеш; версии 6-8 переводятся в текущую без потери калибровки: недостающие поля (за старой
// структурой во флеш - мусор) - по умолчанию; у версии 8 уровень и проценты дежурки те же, пороги - заново от света.
static bool loadCalib() {
    if (boardLoad(s_cal)) {
        return true;
    }
    const uint16_t v = s_cal.version;   // версия во флеш
    if (s_cal.magic != CALIB_MAGIC ||
        (v != CALIB_VERSION_NO_PWM && v != CALIB_VERSION_NO_STANDBY && v != CALIB_VERSION_ONE_STANDBY)) {
        return false;
    }
    if (v == CALIB_VERSION_NO_PWM) {
        s_cal.nightPwmHz = NIGHT_PWM_HZ;
    }
    if (v == CALIB_VERSION_ONE_STANDBY) {
        standbyFromLight(s_cal);
    } else {
        standbyDefaults(s_cal);
    }
    s_cal.version = CALIB_VERSION;
    boardSave(s_cal);
    return true;
}

void setup() {
    if (!loadCalib()) {   // первое включение - без калибровки, по умолчанию; калибровка - долгим нажатием
        calibDefaults(s_cal);
    }
    bool calib = false;   // консоль: команда calib
    if (Stream* con = boardConsoleOpen()) {
        calib = consoleRun(*con, s_cal);
    }
    boardInit();
#ifdef NIGHT_LIGHT
    boardLampFreq(s_cal.nightPwmHz);   // после boardInit: делитель - от рабочих тактов
#endif
    boardLed(Led::Green);  // вспышка при включении: прошивка запустилась (повторяется - МК перезапускается)
    delay(BOOT_FLASH_MS);
    boardLed(Led::Off);
#ifndef NIGHT_LIGHT
    s_supplyCheckMs = millis() - SUPPLY_CHECK_MS;   // первый замер - сразу
#endif
    LOG("Старт, версия %s, калибровка датчика: %s, задержка %u с\n", FW_VERSION,
        sensorCalibrated(s_cal) ? "есть" : "нет (по умолчанию)", s_cal.offDelay);
    startSensorWrite();
    if (calib) {
        startCalibration(false, millis());   // стадии консоль уже показала
    }
}

void loop() {
    const uint32_t now = millis();
    boardLedTick(now);
    buttonTick(now);
#ifndef NIGHT_LIGHT
    supplyTick(now);
#endif
    LdReport r;

    switch (s_mode) {
    case Mode::Normal:
        sensorWriteTick(now);
        normalTick(now);
        stuckCheckTick(now);
#ifdef NIGHT_LIGHT
        tempTick(now);
        standbyTick(now);
#endif
        boardSleep(TICK_MS, s_mode == Mode::Normal);   // начался режим залипания - UART должен принимать
        return;
    case Mode::Stuck:   // лампа - по энергии ворот, выход - OT2 = 0 или калибровка
        stuckTick(now);
#ifdef NIGHT_LIGHT
        tempTick(now);
        standbyTick(now);
#endif
        break;
    case Mode::SetDelay:   // лампа работает как обычно
        normalTick(now);
#ifdef NIGHT_LIGHT
        tempTick(now);
        standbyTick(now);
#endif
        countShowTick(now, s_cal.offDelay / OFF_DELAY_STEP_S, Led::Blue);
        if (!s_buttonDown && now - s_phaseMs >= DELAY_EXIT_MS) {
            finishSetDelay();
        }
        boardSleep(TICK_MS, true);
        return;
#ifdef NIGHT_LIGHT
    case Mode::SetPwm:   // ночник горит на выбранном уровне - слушать писк; присутствие не смотрим
        tempTick(now);
        if (!s_buttonDown) {   // держат после входа - горит зелёный
            countShowTick(now, s_cal.nightPwmHz / HZ_PER_KHZ, Led::Green);
            if (now - s_phaseMs >= PWM_SETUP_EXIT_MS) {
                finishSetPwm();
            }
        }
        boardSleep(TICK_MS, true);   // идёт ШИМ - boardSleep сам спит без остановки тактов
        return;
#endif
    case Mode::CalLeave:
        if (now - s_phaseMs >= CAL_LEAVE_S * 1000) {
            startAbsence(now);
        }
        boardSleep(TICK_MS, true);   // датчик ещё не слушаем
        return;
    case Mode::CalAbsence:
        sampleLight(now, (CAL_SCAN_S + CAL_SCAN_MARGIN_S) * 1000);
        while (ldPoll(r)) {
            s_scanReports++;
            if (r.type == 3) {
                s_scanProgress = r.progress;
                LOG("автокалибровка: %u\n", r.progress);
                if (r.progress / CAL_PROGRESS_STEP != s_progressStep) {   // progress - % (перехват; табл. 2-1: % * 100)
                    s_progressStep = (uint8_t)(r.progress / CAL_PROGRESS_STEP);
                    calMsg("автокалибровка датчика %u %%", r.progress);
                }
            }
        }
        if (now - s_phaseMs >= (CAL_SCAN_S + CAL_SCAN_MARGIN_S) * 1000) {
            startPresence(now);
        }
        break;
    case Mode::CalPresence:
        if (!s_sensorReady) {   // пауза и попытки: отчёты без порогов и энергий не нужны
            if (now - s_readTryMs >= (s_readTries == CAL_READ_TRIES ? CAL_READ_PAUSE_MS : CAL_READ_RETRY_MS)) {
                s_readTryMs = now;
                readScanThresholds(now);
            }
            while (ldPoll(r)) {
            }
            break;
        }
        sampleLight(now, CAL_PRESENCE_S * 1000);
        while (ldPoll(r)) {
            countReport(r);
        }
        if (now - s_phaseMs >= CAL_PRESENCE_S * 1000) {
            finishCalibration();
        }
        break;
    case Mode::Result:
        resultTick(now);
        boardSleep(TICK_MS, true);
        return;
    }
    boardSleep(TICK_MS, false);   // калибровка: UART должен принимать - без остановки тактов
}
