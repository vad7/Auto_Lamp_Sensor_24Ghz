// Сценарии симулятора: мир (человек, «банка», свет, кнопка, консоль) по времени + проверки поведения прошивки.
// Запуск: sim_<вариант>.exe list | <сценарий>. Сборка и прогон всех - Scripts/sim_test.py.
#include "sim.h"
#include "console.h"

void setup();
void loop();

static int s_checks = 0;   // проверок
static int s_fails = 0;    // проваленных

static void check(bool ok, int line, const char* fmt, ...) __attribute__((format(gnu_printf, 3, 4)));
static void check(bool ok, int line, const char* fmt, ...) {
    char text[300];
    va_list a;
    va_start(a, fmt);
    vsnprintf(text, sizeof(text), fmt, a);
    va_end(a);
    s_checks++;
    s_fails += ok ? 0 : 1;
    fprintf(stdout, "[%10.3f] %s (строка %d): %s\n", g_simUs / 1e6, ok ? "ПРОВЕРКА ок" : "ПРОВАЛ", line, text);
}
#define CHECK(cond, ...) check((cond), __LINE__, __VA_ARGS__)

constexpr uint32_t NONE = UINT32_MAX;   // события не было
#ifdef NIGHT_LIGHT
constexpr int LAMP_ON_PCT = NIGHT_LEVELS[NIGHT_DEFAULT_LEVEL];   // ночник: уровень по умолчанию
#else
constexpr int LAMP_ON_PCT = LAMP_FULL_PCT;
#endif

static void runTo(uint32_t ms) {
    while (simNowMs() < ms) {
        loop();
    }
}

// Работать, пока не выполнится условие (не дольше maxMs); время выполнения или NONE.
template <class F> static uint32_t runUntil(F cond, uint32_t maxMs) {
    const uint32_t end = simNowMs() + maxMs;
    while (simNowMs() < end) {
        if (cond()) {
            return simNowMs();
        }
        loop();
    }
    return NONE;
}

static void press(uint32_t holdMs) {   // нажать BOOT на holdMs (с работой прошивки)
    simSay("BOOT нажата на %u мс", holdMs);
    g_world.button = true;
    runTo(simNowMs() + holdMs);
    g_world.button = false;
}

static int lampAt(uint32_t ms) {
    int v = 0;
    for (size_t i = 0; i < g_lampLog.size() && g_lampLog[i].ms <= ms; i++) {
        v = g_lampLog[i].value;
    }
    return v;
}
static uint32_t firstLamp(bool on, uint32_t from) {   // первое включение / выключение не раньше from
    for (size_t i = 0; i < g_lampLog.size(); i++) {
        if (g_lampLog[i].ms >= from && (g_lampLog[i].value > 0) == on) {
            return g_lampLog[i].ms;
        }
    }
    return NONE;
}
static uint32_t firstLampAtLeast(int pct, uint32_t from) {   // первая яркость не меньше pct не раньше from
    for (size_t i = 0; i < g_lampLog.size(); i++) {
        if (g_lampLog[i].ms >= from && g_lampLog[i].value >= pct) {
            return g_lampLog[i].ms;
        }
    }
    return NONE;
}
static int ledAt(uint32_t ms) {
    int v = 0;
    for (size_t i = 0; i < g_ledLog.size() && g_ledLog[i].ms <= ms; i++) {
        v = g_ledLog[i].value;
    }
    return v;
}
static uint32_t firstLed(Led led, uint32_t from) {
    for (size_t i = 0; i < g_ledLog.size(); i++) {
        if (g_ledLog[i].ms >= from && g_ledLog[i].value == (int)led) {
            return g_ledLog[i].ms;
        }
    }
    return NONE;
}
static uint32_t cmdTime(uint16_t cmd, uint32_t from, bool last = false) {   // первая (последняя) команда с from
    uint32_t t = NONE;
    for (size_t i = 0; i < g_sensor.cmdLog.size(); i++) {
        if (g_sensor.cmdLog[i].cmd == cmd && g_sensor.cmdLog[i].ms >= from) {
            t = g_sensor.cmdLog[i].ms;
            if (!last) {
                break;
            }
        }
    }
    return t;
}
static int cmdCount(uint16_t cmd, uint32_t from, uint32_t to) {
    int n = 0;
    for (size_t i = 0; i < g_sensor.cmdLog.size(); i++) {
        n += g_sensor.cmdLog[i].cmd == cmd && g_sensor.cmdLog[i].ms >= from && g_sensor.cmdLog[i].ms < to;
    }
    return n;
}
static int countIn(const std::string& text, const char* what) {
    int n = 0;
    for (size_t p = text.find(what); p != std::string::npos; p = text.find(what, p + 1)) {
        n++;
    }
    return n;
}
static bool near(uint32_t t, uint32_t expect, uint32_t tol) {
    return t != NONE && t + tol >= expect && t <= expect + tol;
}

// Сохранённая калибровка: пороги 35 / 30 дБ, дальние ворота 6, свет темно 300 / светло 2300.
static void presetCalib() {
    Calib c;
    calibDefaults(c);
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        c.trig[g] = 35;
        c.hold[g] = 30;
    }
    c.farGate = 6;
    c.ldrInputMode = LDR_DARK_LOW;
    c.lightThreshold = 1300;
    c.lightDarkLevel = 300;
    c.lightLitLevel = 2300;
    g_flash = c;
    g_flashValid = true;
}

// Флеш от прошивки с калибровкой версии 6: частоты ШИМ в структуре нет - на её месте стёртая флеш.
static void scCalibFromV6() {
    presetCalib();
    g_flash.version = CALIB_VERSION_NO_PWM;
    g_flash.nightPwmHz = 0xFFFFFFFF;
    setup();
    runTo(5000);
    CHECK(g_flash.version == CALIB_VERSION && g_flash.nightPwmHz == NIGHT_PWM_HZ && g_saves == 1,
          "переведена в версию %u, pwm %u Гц, записей во флеш %d", g_flash.version, g_flash.nightPwmHz, g_saves);
    CHECK(sensorCalibrated(g_flash) && g_flash.farGate == 6 && g_flash.lightDarkLevel == 300,
          "калибровка сохранилась (пороги, дальние ворота %u, темно %u)", g_flash.farGate, g_flash.lightDarkLevel);
    CHECK(g_sensor.farGate == 6, "датчику записаны дальние ворота калибровки (%u)", g_sensor.farGate);
#ifdef NIGHT_LIGHT
    CHECK(g_lampPwmHz == NIGHT_PWM_HZ, "ШИМ %u Гц (ждали %u)", g_lampPwmHz, NIGHT_PWM_HZ);
#endif
}

// Флеш от прошивки с калибровкой версии 7: полей дежурки нет - на их месте мусор; пороги дежурки - от калибровки
// света (темно 300, светло 2300: вкл 800, погасить 1000 на всех уровнях), основной при ней - как light.
static bool standbyFromLightOk() {
    bool ok = g_flash.standbyOnThreshold == 800;
    for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {
        ok = ok && g_flash.standbyOffThreshold[i] == 1000 && g_flash.lightThresholdStandby[i] == g_flash.lightThreshold &&
             g_flash.lightStandbyLevel[i] == 0;
    }
    return ok;
}

// Флеш версии from: калибровка датчика и света сохраняется, пороги дежурки - заново от света.
static void calibFromOld(uint16_t from) {
    presetCalib();
    g_flash.version = from;
    g_flash.nightPwmHz = 4000;
    const size_t keep = from == CALIB_VERSION_ONE_STANDBY ? offsetof(Calib, standbyOnThreshold)   // уровень, проценты
                                                          : offsetof(Calib, standbyLevel);
    if (from == CALIB_VERSION_ONE_STANDBY) {
        g_flash.standbyLevel = 2;
        g_flash.standbyPct[0] = 2;
    }
    memset((uint8_t*)&g_flash + keep, 0xFF, sizeof(Calib) - keep);   // за старой структурой - мусор
    setup();
    runTo(5000);
    CHECK(g_flash.version == CALIB_VERSION && g_flash.nightPwmHz == 4000 && g_saves == 1,
          "переведена в версию %u, pwm %u Гц сохранена, записей %d", g_flash.version, g_flash.nightPwmHz, g_saves);
    if (from == CALIB_VERSION_ONE_STANDBY) {
        CHECK(g_flash.standbyLevel == 2 && g_flash.standbyPct[0] == 2, "дежурка: уровень %u и проценты сохранились",
              g_flash.standbyLevel);
    } else {
        CHECK(g_flash.standbyLevel == 0 && !memcmp(g_flash.standbyPct, STANDBY_LEVELS, sizeof(STANDBY_LEVELS)),
              "дежурка выкл, уровни по умолчанию");
    }
    CHECK(standbyFromLightOk(), "пороги дежурки: вкл %u, погасить %u, основной при ней %u", g_flash.standbyOnThreshold,
          g_flash.standbyOffThreshold[0], g_flash.lightThresholdStandby[0]);
    CHECK(sensorCalibrated(g_flash) && g_flash.farGate == 6, "калибровка датчика сохранилась");
}
static void scCalibFromV7() { calibFromOld(CALIB_VERSION_NO_STANDBY); }
static void scCalibFromV8() { calibFromOld(CALIB_VERSION_ONE_STANDBY); }

static void checkSensorIdle(const char* when) {   // датчик после работы с ним: как в обычной работе
    CHECK(!g_sensor.configMode && !g_sensor.standard && !g_sensor.uartOpen,
          "%s: датчик вне режима команд (%d), минимальный отчёт (стандартный %d), UART закрыт (%d)", when,
          g_sensor.configMode, g_sensor.standard, g_sensor.uartOpen);
}

// ======================================================================= сценарии

// Первое включение (калибровки нет): настройки в датчик, человек в темноте - лампа, вышел - выкл через задержку + minon.
static void scBootDefault() {
    setup();
    runTo(5000);
    CHECK(g_sensor.offDelayS == OFF_DELAY_S && g_sensor.farGate == 12, "датчику записана задержка %u с, дальние "
          "ворота не тронуты (%u)", g_sensor.offDelayS, g_sensor.farGate);
    CHECK(g_sensor.responseSpeed == SENSOR_RESPONSE_SPEED, "скорость отклика датчика %u (ждали %u)",
          g_sensor.responseSpeed, SENSOR_RESPONSE_SPEED);
    checkSensorIdle("после записи настроек");
    runTo(20000);
    simSay("человек вошёл (ворота 3)");
    g_sensor.personGate = 3;
    runTo(60000);
    simSay("человек вышел");
    g_sensor.personGate = -1;
    runTo(130000);
    const uint32_t on = firstLamp(true, 0);
    CHECK(near(on, 20300, 300), "лампа вкл через %d мс после входа", (int)(on - 20000));
    CHECK(lampAt(on) == LAMP_ON_PCT, "яркость %d %% (ждали %d)", lampAt(on), LAMP_ON_PCT);
    const uint32_t expect = 60000 + OFF_DELAY_S * 1000 + MIN_ON_DEFAULT_S * 1000;   // OT2 = 0 через задержку, + minon
    const uint32_t off = firstLamp(false, on);
    CHECK(near(off, expect, 600), "лампа выкл в %.1f с (ждали ~%.1f: выход + задержка %u + minon %u)", off / 1e3,
          expect / 1e3, OFF_DELAY_S, MIN_ON_DEFAULT_S);
    CHECK(g_saves == 0, "флеш не записывалась (%d)", g_saves);
    CHECK(firstLed(Led::NoSensor, 0) == NONE, "индикатор «нет датчика» не включался");
}

// Светло - лампа не горит; выключили свет - лампа включается при повторном замере.
static void scBootBright() {
    g_world.ambient = 3000;
    setup();
    runTo(20000);
    simSay("человек вошёл, светло");
    g_sensor.personGate = 3;
    runTo(40000);
    CHECK(firstLamp(true, 0) == NONE, "светло: лампа не включалась");
    simSay("свет в ванной выключили");
    g_world.ambient = 300;
    runTo(50000);
    const uint32_t on = firstLamp(true, 0);
    CHECK(on != NONE && on <= 40000 + LIGHT_RECHECK_MS + 300, "лампа вкл в %.1f с (повторный замер раз в %u с)",
          on / 1e3, LIGHT_RECHECK_MS / 1000);
}

// Калибровка кнопкой: пауза, шаг 1 (автокалибровка), пауза и чтение порогов, шаг 2 (человек ходит), итог.
// silenceMs - тишина датчика после 100 % (перехват ~3 с, на железе до ~9 с); entryMs - человек входит через столько
// после лампы шага 2.
static void calibByButton(uint32_t silenceMs, uint32_t entryMs = 1000) {
    g_sensor.silenceMs = silenceMs;
#ifdef NIGHT_LIGHT
    g_world.lampLight = 2000;   // свет ночника на фоторезисторе: дежурка 5 % - +100
#endif
    setup();
    runTo(5000);
    press(11000);   // 10 с - калибровка
    const uint32_t start = firstLed(Led::CalLeave, 0);
    CHECK(near(start, 5000 + CAL_PRESS_MS, 100), "калибровка началась через %d мс удержания", (int)(start - 5000));
    // Человек входит, когда загорелась лампа шага 2 (сигнал «войти»; у ночника до неё - замер с дежуркой, слабее)
    runUntil([] { return lampAt(simNowMs()) >= LAMP_ON_PCT; }, 200000);
    const uint32_t lampOn = firstLampAtLeast(LAMP_ON_PCT, start);   // из журнала (runUntil видит на проход позже)
    runTo(lampOn + entryMs);
    simSay("человек вошёл и ходит (ворота 1..4)%s", LAMP_ON_PCT == LAMP_FULL_PCT ? "" : ", включил свет в ванной");
    g_sensor.personMoving = true;
#ifdef NIGHT_LIGHT
    g_world.ambient = 2300;
#endif
    const uint32_t autoCmd = cmdTime(0x0009, start);
    CHECK(near(autoCmd, start + CAL_LEAVE_S * 1000, 300), "шаг 1 (0x0009) через %d мс после начала",
          (int)(autoCmd - start));
    const uint32_t step1End = autoCmd + (CAL_SCAN_S + CAL_SCAN_MARGIN_S) * 1000;
#ifdef NIGHT_LIGHT
    // Дежурка в шаге 1: в конце шага по CAL_STANDBY_WINDOW_MS каждый уровень, затем погашена до шага 2
    const uint32_t sbOn = firstLamp(true, start);
    CHECK(near(sbOn, step1End - STANDBY_LEVEL_COUNT * CAL_STANDBY_WINDOW_MS, 100), "шаг 1: дежурка за %.1f с до конца",
          (step1End - sbOn) / 1e3);
    for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {
        const uint32_t mid = sbOn + i * CAL_STANDBY_WINDOW_MS + CAL_STANDBY_WINDOW_MS / 2;   // середина окна уровня
        CHECK(lampAt(mid) == STANDBY_LEVELS[i], "шаг 1: окно %u - дежурка %d %%", i, lampAt(mid));
    }
    CHECK(near(firstLamp(false, sbOn), step1End, 100) && lampAt(lampOn - 1) == 0, "дежурка погасла в конце шага 1");
#else
    CHECK(lampAt(lampOn - 1) == 0 && firstLamp(true, start) == lampOn, "лампа не горела с начала калибровки до "
          "шага 2");
#endif
    const uint32_t lastRead = cmdTime(0x0073, start, true);
    CHECK(lampOn >= step1End + CAL_READ_PAUSE_MS && lastRead < lampOn,
          "лампа шага 2 в %.1f с: после паузы %u с и чтения порогов (последнее 0x0073 в %.1f с)", lampOn / 1e3,
          CAL_READ_PAUSE_MS / 1000, lastRead / 1e3);
    CHECK(lampOn >= g_sensor.silenceUntilUs / 1000, "лампа - после тишины датчика (до %.1f с)",
          g_sensor.silenceUntilUs / 1e6);
    const uint32_t green = firstLed(Led::Green, step1End);
    CHECK(near(green, lampOn, 50), "индикатор зелёный вместе с лампой (%.3f с)", green / 1e3);
    runTo(lampOn + CAL_PRESENCE_S * 1000 + 3000);
    CHECK(g_saves == 1, "калибровка сохранена (записей %d)", g_saves);
    CHECK(firstLed(Led::Blue, lampOn) != NONE, "итог: красный / синий / зелёный");
    CHECK(g_flash.farGate == 6, "дальние ворота %u (человек на 1..4, соседние ворота 5 -> 6)", g_flash.farGate);
    CHECK(g_flash.ldrInputMode == LDR_DARK_LOW, "фоторезистор: прямая кривая (%u)", g_flash.ldrInputMode);
    CHECK(near(g_flash.lightDarkLevel, 300, 5) && near(g_flash.lightLitLevel, 2300, 5) &&
          near(g_flash.lightThreshold, 1300, 5), "свет: темно %u, светло %u, порог %u", g_flash.lightDarkLevel,
          g_flash.lightLitLevel, g_flash.lightThreshold);
#ifdef NIGHT_LIGHT
    // Дежурка 1 / 3 / 5 % - свет +20 / +60 / +100: вкл темнее 300 + 25 % от 2000 = 800; погасить светлее 800 + свет +
    // 10 % (200) = 1020 / 1060 / 1100; основной при ней - 1300 + свет = 1320 / 1360 / 1400
    CHECK(near(g_flash.standbyOnThreshold, 800, 5), "дежурка: sbon %u", g_flash.standbyOnThreshold);
    for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {
        const uint32_t add = 2000u * STANDBY_LEVELS[i] / 100;   // свет дежурки на фоторезисторе в модели
        CHECK(near(g_flash.lightStandbyLevel[i], 300 + add, 5) && near(g_flash.standbyOffThreshold[i], 1000 + add, 5) &&
              near(g_flash.lightThresholdStandby[i], 1300 + add, 5),
              "дежурка %u %%: темно с ней %u, sboff %u, lightsb %u", STANDBY_LEVELS[i], g_flash.lightStandbyLevel[i],
              g_flash.standbyOffThreshold[i], g_flash.lightThresholdStandby[i]);
    }
#endif
    CHECK(!memcmp(g_flash.trig, g_sensor.trig, sizeof(g_flash.trig)) &&
          !memcmp(g_flash.hold, g_sensor.hold, sizeof(g_flash.hold)), "во флеш - пороги из датчика (ворота 0: %u / %u)",
          g_flash.trig[0], g_flash.hold[0]);
    CHECK(g_sensor.farGate == 6 && g_sensor.offDelayS == OFF_DELAY_S, "датчику записаны дальние ворота %u, задержка %u",
          g_sensor.farGate, g_sensor.offDelayS);
    checkSensorIdle("после калибровки");
    // После калибровки - обычная работа
    const uint32_t t = simNowMs();
#ifdef NIGHT_LIGHT
    runTo(t + 5000);
    simSay("свет в ванной выключили");
    g_world.ambient = 300;
    const uint32_t nightOn = runUntil([] { return lampAt(simNowMs()) > 0; }, LIGHT_RECHECK_MS + 1000);
    CHECK(nightOn != NONE, "ночник: свет выключили - ночник вкл (%.1f с)", nightOn / 1e3);
#endif
    runTo(t + 10000);
    simSay("человек вышел");
    g_sensor.personMoving = false;
    const uint32_t left = simNowMs();
    runTo(left + (OFF_DELAY_S + MIN_ON_DEFAULT_S) * 1000 + 3000);
    const uint32_t off = firstLamp(false, left);
    CHECK(near(off, left + (OFF_DELAY_S + MIN_ON_DEFAULT_S) * 1000, 800), "лампа выкл через %.1f с после выхода",
          (off - left) / 1e3);
}

static void scCalibOk() { calibByButton(3000); }

// Человек входит через 10 с после лампы шага 2: видно 2/3 кадров - хватает (CAL_PRESENT_PCT 50 %).
static void scCalibLateEntry() { calibByButton(3000, 10000); }

// Датчик после автокалибровки молчит 9 с (как на железе) - чтение порогов с повторами.
static void scCalibSlowSensor() {
    calibByButton(9000);
    CHECK(countIn(g_serialOut, "повтор") >= 1, "были повторы чтения порогов (%d)", countIn(g_serialOut, "повтор"));
}

// Датчик после автокалибровки не отвечает совсем: ошибка после всех попыток, лампа не горит, затем «нет датчика».
static void scCalibDeadSensor() {
    g_sensor.deadAfterAuto = true;
    setup();
    runTo(5000);
    press(11000);
    runTo(230000);
    CHECK(g_saves == 0, "калибровка не сохранена (%d)", g_saves);
    CHECK(firstLampAtLeast(LAMP_ON_PCT, 15000) == NONE, "лампа шага 2 не включалась");
    CHECK(countIn(g_serialOut, "ОШИБКА") >= 1, "сообщение об ошибке в USB");
    const int tries = cmdCount(0x00FF, 150000, 175000);
    CHECK(tries >= CAL_READ_TRIES, "попыток чтения порогов %d (ждали >= %u)", tries, CAL_READ_TRIES);
    CHECK(ledAt(simNowMs()) == (int)Led::NoSensor, "индикатор «нет датчика» (%s)", ledName(ledAt(simNowMs())));
    CHECK(g_deepDrops == 0 && !g_sensor.uartOpen, "UART закрыт");
}

// Шаг 2 без человека: ошибка, в датчик - прежняя калибровка.
static void scCalibNoPerson() {
    presetCalib();
    setup();
    runTo(5000);
    press(11000);
    runTo(200000);
    CHECK(g_saves == 0, "калибровка не сохранена (%d)", g_saves);
    CHECK(countIn(g_serialOut, "плохо видит человека") >= 1, "причина: датчик плохо видит человека");
    CHECK(g_sensor.trig[0] == 35 && g_sensor.hold[0] == 30 && g_sensor.farGate == 6,
          "в датчике прежние пороги %u / %u и дальние ворота %u", g_sensor.trig[0], g_sensor.hold[0], g_sensor.farGate);
    checkSensorIdle("после ошибки");
    CHECK(ledAt(simNowMs()) == (int)Led::Off, "индикатор погашен");
}

// Залипание: «банка» держит OT2; через stuck мин лампа выкл, вспышки; человек - лампа по энергии; убрали - обычно.
static void scStuckJar() {
    presetCalib();
    setup();
    runTo(10000);
    simSay("банка у датчика (ворота 2)");
    g_sensor.jarGate = 2;
    runUntil([] { return lampAt(simNowMs()) > 0; }, 2000);
    const uint32_t on = firstLamp(true, 0);
    CHECK(on != NONE, "банка: лампа вкл (%.1f с)", on / 1e3);
    const uint32_t stuckMs = STUCK_MINUTES_DEFAULT * 60000UL;
    runTo(on + stuckMs - 5000);
    CHECK(firstLed(Led::StuckFlash, 0) == NONE && lampAt(simNowMs()) > 0, "до %u мин - обычная работа, лампа горит",
          STUCK_MINUTES_DEFAULT);
    runUntil([] { return firstLed(Led::StuckFlash, 0) != NONE; }, 5000 + STUCK_LIGHT_CHECK_MS + 5000);
    const uint32_t stuck = firstLed(Led::StuckFlash, 0);
    // Свет начала отсчёта меряется до включения лампы: лампа 230 В меняет свет - первый замер (через
    // STUCK_LIGHT_CHECK_MS) начинает отсчёт заново, залипание до STUCK_LIGHT_CHECK_MS позже
    CHECK(stuck != NONE && stuck >= on + stuckMs && stuck <= on + stuckMs + STUCK_LIGHT_CHECK_MS + 100,
          "залипание через %.2f мин", (stuck - on) / 60e3);
    CHECK(firstLamp(false, on) == stuck, "лампа выкл при залипании");
    CHECK(g_sensor.uartOpen && g_sensor.standard && !g_sensor.configMode, "датчик в стандартном отчёте, UART открыт");
    runTo(stuck + 60000);
    CHECK(firstLamp(true, stuck) == NONE, "банка без человека: лампа не включалась (фон + %u дБ)",
          STUCK_DELTA_DB_DEFAULT);
    simSay("человек вошёл (ворота 4)");
    g_sensor.personGate = 4;
    const uint32_t t = simNowMs();
    runTo(t + 2000);
    const uint32_t on2 = firstLamp(true, t);
    CHECK(near(on2, t + 300, 300), "залипание, человек: лампа вкл через %d мс", (int)(on2 - t));
    runTo(t + 60000);
    simSay("человек вышел");
    g_sensor.personGate = -1;
    const uint32_t left = simNowMs();
    runTo(left + MIN_ON_DEFAULT_S * 1000 + 2000);
    const uint32_t off2 = firstLamp(false, left);
    CHECK(near(off2, left + MIN_ON_DEFAULT_S * 1000, 600), "лампа выкл через %.1f с после выхода (minon %u)",
          (off2 - left) / 1e3, MIN_ON_DEFAULT_S);
    runTo(left + 60000);
    simSay("банку убрали");
    g_sensor.jarGate = -1;
    const uint32_t removed = simNowMs();
    runTo(removed + OFF_DELAY_S * 1000 + 2000);
    const uint32_t normal = firstLed(Led::Off, removed);
    CHECK(near(normal, removed + OFF_DELAY_S * 1000, 600), "выход из залипания через %.1f с (OT2 = 0)",
          (normal - removed) / 1e3);
    checkSensorIdle("после залипания");
    runTo(simNowMs() + 10000);
    simSay("человек вошёл");
    g_sensor.personGate = 3;
    const uint32_t t3 = simNowMs();
    runTo(t3 + 2000);
    CHECK(near(firstLamp(true, t3), t3 + 300, 300), "обычная работа: лампа вкл");
}

// Лампа включается много раз с разной фазой миллисекунды: режим залипания не должен включаться сразу
// (отсчёт залипания в setLamp по millis(), сравнение в stuckCheckTick по now того же прохода loop).
static void scStuckFalseStart() {
    setup();   // без калибровки света: полоса 0 - залипание только по времени
    runTo(5000);
    for (int i = 0; i < 40; i++) {
        const uint64_t phase = (uint64_t)i * 25;   // фаза прохода loop внутри миллисекунды, мкс: 0..975
        simAdvanceUs((phase + 1000 - g_simUs % 1000) % 1000);
        g_sensor.personGate = 3;
        runTo(simNowMs() + 2000);
        g_sensor.personGate = -1;
        runTo(simNowMs() + (OFF_DELAY_S + MIN_ON_DEFAULT_S) * 1000 + 2000);
    }
    int ons = 0;
    for (size_t i = 0; i < g_lampLog.size(); i++) {
        ons += g_lampLog[i].value > 0;
    }
    const uint32_t stuck = firstLed(Led::StuckFlash, 0);
    CHECK(stuck == NONE, "включений лампы %d, ложных залипаний нет (первое в %.3f с)", ons,
          stuck == NONE ? 0.0 : stuck / 1e3);
}

// Свет меняется (включили свет в ванной) - отсчёт залипания заново.
static void scStuckLightChange() {
    presetCalib();
    setup();
    runTo(10000);
    simSay("банка у датчика (ворота 2)");
    g_sensor.jarGate = 2;
    runUntil([] { return lampAt(simNowMs()) > 0; }, 2000);
    const uint32_t on = firstLamp(true, 0);
    runTo(on + 40 * 60000UL);
    simSay("свет +100 отсчётов (полоса %u %% = %u)", STUCK_LIGHT_PCT_DEFAULT, 2000 * STUCK_LIGHT_PCT_DEFAULT / 100);
    g_world.ambient += 100;
    const uint32_t changed = simNowMs();
    runTo(changed + STUCK_MINUTES_DEFAULT * 60000UL - 5000);
    CHECK(firstLed(Led::StuckFlash, 0) == NONE, "свет менялся: залипания нет через %u мин от включения",
          (simNowMs() - on) / 60000);
    runTo(changed + STUCK_MINUTES_DEFAULT * 60000UL + STUCK_LIGHT_CHECK_MS + 1000);
    const uint32_t stuck = firstLed(Led::StuckFlash, 0);
    CHECK(stuck != NONE && stuck >= changed + STUCK_MINUTES_DEFAULT * 60000UL,
          "залипание через %.1f мин после смены света", (stuck - changed) / 60e3);
}

// Залипание, затем калибровка кнопкой: банка уходит в фон, после неё OT2 по человеку.
static void scStuckThenCalib() {
    presetCalib();
    setup();
    runTo(10000);
    simSay("банка у датчика (ворота 2)");
    g_sensor.jarGate = 2;
    const uint32_t stuck = runUntil([] { return firstLed(Led::StuckFlash, 0) != NONE; }, 3700000);
    CHECK(stuck != NONE, "залипание");
    runTo(stuck + 10000);
    press(11000);
    CHECK(!g_sensor.uartOpen && firstLed(Led::CalLeave, stuck) != NONE, "калибровка: UART закрыт, пауза выхода");
    const uint32_t lampOn = runUntil([] { return lampAt(simNowMs()) >= LAMP_ON_PCT; }, 200000);   // шаг 2
    runTo(lampOn + 1000);
    simSay("человек вошёл и ходит");
    g_sensor.personMoving = true;
#ifdef NIGHT_LIGHT
    g_world.ambient = 2300;
#endif
    runTo(lampOn + CAL_PRESENCE_S * 1000 + 3000);
    CHECK(g_saves == 1, "калибровка сохранена (%d)", g_saves);
    CHECK(g_sensor.trig[2] > 50, "банка в фоне: порог ворот 2 = %u дБ", g_sensor.trig[2]);
    checkSensorIdle("после калибровки");
    simSay("человек вышел, банка осталась");
    g_sensor.personMoving = false;
#ifdef NIGHT_LIGHT
    g_world.ambient = 300;
#endif
    const uint32_t left = simNowMs();
    runTo(left + OFF_DELAY_S * 1000 + 2000);
    CHECK(!g_sensor.ot2, "OT2 = 0: банка ниже новых порогов");
}

// Консоль: команды залипания и неверные значения.
static void scConsoleSettings() {
    g_world.consoleJumper = true;
    g_world.consoleInput = {{500, "stuckpct 5\r\n"}, {1000, "stuckdb 4\r\n"}, {1500, "stuckdb 0\r\n"},
                            {2000, "ldr 3\r\n"},     {2500, "stuck 30\r\n"},  {3000, "minon 45\r\n"},
                            {3500, "show\r\n"},      {4000, "exit\r\n"}};
    setup();
    runTo(6000);
    CHECK(g_flash.stuckLightPct == 5 && g_flash.stuckDeltaDb == 4 && g_flash.stuckMinutes == 30 && g_flash.minOnS == 45,
          "сохранено: stuckpct %u, stuckdb %u, stuck %u, minon %u", g_flash.stuckLightPct, g_flash.stuckDeltaDb,
          g_flash.stuckMinutes, g_flash.minOnS);
    CHECK(g_flash.ldrInputMode == LDR_DIVIDER_MODE, "ldr 3 не принят (ldr %u)", g_flash.ldrInputMode);
    CHECK(countIn(g_consoleOut, "неверная команда") == 2, "неверных команд 2 (%d)",
          countIn(g_consoleOut, "неверная команда"));
    CHECK(countIn(g_consoleOut, "залипание: stuck 30 мин, stuckpct 5 %, stuckdb 4 дБ") == 1, "show: строка залипания");
    CHECK(countIn(g_consoleOut, "Выход из консоли, работа") == 1, "выход из консоли");
}

#ifdef NIGHT_LIGHT
// Настройка частоты ШИМ кнопкой: из настройки задержки удержание 10 с - зелёный, ночник горит на уровне; 3500 Гц -> 3 кГц;
// короткое +1, длинное -1, по кругу (1 - 1 = 25, 25 + 1 = 1); выход через 60 с - сохранено, ночник погас.
static void scSetPwm() {
    presetCalib();
    setup();
    runTo(5000);
    press(6000);                                   // настройка задержки
    runTo(simNowMs() + 2000);
    const uint32_t holdFrom = simNowMs();
    press(CAL_PRESS_MS + 1000);                    // удержание - настройка частоты
    const uint32_t green = firstLed(Led::Green, holdFrom);
    CHECK(green != NONE && near(green, holdFrom + CAL_PRESS_MS, 100), "зелёный через %d мс удержания",
          (int)(green - holdFrom));
    CHECK(lampAt(simNowMs()) == LAMP_ON_PCT, "ночник горит на уровне: %d %% (ждали %d)", lampAt(simNowMs()),
          LAMP_ON_PCT);
    CHECK(g_lampPwmHz == 3000, "вход: 3500 Гц -> %u Гц (целые кГц)", g_lampPwmHz);
    CHECK(cmdTime(0x0009, holdFrom) == NONE, "калибровка не запущена");
    const uint32_t steps[] = {300, 1500, 1500, 1500, 1500, 300, 300};   // 4, 3, 2, 1, 25, 1, 2 кГц
    for (uint32_t ms : steps) {
        runTo(simNowMs() + 1000);
        press(ms);
    }
    runTo(simNowMs() + 100);   // отпускание прошивка видит на следующем опросе
    CHECK(g_lampPwmHz == 2000, "после шагов %u Гц (ждали 2000: через 25 по кругу)", g_lampPwmHz);
    const uint32_t lastPress = simNowMs();
    runTo(lastPress + PWM_SETUP_EXIT_MS + 2000);
    CHECK(g_flash.nightPwmHz == 2000, "сохранено %u Гц", g_flash.nightPwmHz);
    const uint32_t off = firstLamp(false, lastPress + 1);
    CHECK(off != NONE && near(off, lastPress + PWM_SETUP_EXIT_MS, 100), "ночник погас при выходе через %d мс",
          (int)(off - lastPress));
    checkSensorIdle("после настройки частоты");
}

// Консоль ночника: on P [F] - частота ШИМ по умолчанию, заданная, вне пределов; pwm N - сохраняется и действует
// после выхода; pct - все уровни одной строкой (мало, много чисел, 0 - неверно).
static void scConsoleNight() {
    g_world.consoleJumper = true;
    g_world.consoleInput = {{500, "on 5\r\n"},  {1000, "on 10 2000\r\n"}, {1500, "on 10 50\r\n"},
                            {2000, "pwm 50\r\n"}, {2500, "pwm 3000\r\n"},    {3000, "on 5\r\n"},
                            {3500, "pct 3 6 12 25 50\r\n"}, {4000, "pct 5 10\r\n"}, {4500, "pct 1 2 3 4 5 6\r\n"},
                            {5000, "pct 0 10 20 35 60\r\n"}, {5500, "exit\r\n"}};
    setup();
    runTo(7000);
    const uint8_t pct[NIGHT_LEVEL_COUNT] = {3, 6, 12, 25, 50};   // ожидаемые уровни после pct
    CHECK(!memcmp(g_flash.nightPct, pct, sizeof(pct)), "pct сохранены: %u %u %u %u %u", g_flash.nightPct[0],
          g_flash.nightPct[1], g_flash.nightPct[2], g_flash.nightPct[3], g_flash.nightPct[4]);
    char expect[64];   // ожидаемая строка ответа на первое on 5
    snprintf(expect, sizeof(expect), "ночник 5 %%, ШИМ %lu Гц", (unsigned long)NIGHT_PWM_HZ);
    CHECK(countIn(g_consoleOut, expect) == 1, "on 5: частота по умолчанию (%s)", expect);
    CHECK(countIn(g_consoleOut, "ночник 10 %, ШИМ 2000 Гц") == 1, "on 10 2000: заданная частота");
    CHECK(countIn(g_consoleOut, "неверная команда") == 5, "on 10 50, pwm 50, три pct - неверные (%d)",
          countIn(g_consoleOut, "неверная команда"));
    CHECK(countIn(g_consoleOut, "ночник 5 %, ШИМ 3000 Гц") == 1, "on 5 после pwm 3000 - частота из настройки");
    CHECK(g_flash.nightPwmHz == 3000, "pwm сохранена (%u)", g_flash.nightPwmHz);
    CHECK(g_lampPwmHz == 3000, "после выхода из консоли ШИМ %u Гц (ждали 3000)", g_lampPwmHz);
}
#endif

// Консоль: пороги из вывода show («G: T / H»): построчная вставка (с отступом и без), склеенная терминалом в одну
// строку, неверные строки (ворота 16, обрезанная, лишнее) - пороги не меняются; после выхода - пороги в датчике.
static void scConsoleThresholds() {
    g_world.consoleJumper = true;
    std::string lines;    // вставка построчно, как скопировано из show: пороги 60-g / 50-g
    std::string joined;   // все ворота одной строкой (терминал выкинул переводы строк): пороги 70-g / 40-g
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        char row[24];     // строка ворот
        snprintf(row, sizeof(row), "%2u: %u / %u\r\n", g, 60 - g, 50 - g);
        lines += row;
        snprintf(row, sizeof(row), "%2u: %u / %u  ", g, 70 - g, 40 - g);   // два пробела - как склеил терминал
        joined += row;
    }
    g_world.consoleInput = {{500, lines},
                            {3000, "show\r\n"},
                            {3500, "16: 40 / 30\r\n"},
                            {4000, "0: 53 / 47  1: 52 / 46  2: 47 / 44  3: 46 /\r\n"},
                            {4500, "3: 40 / 30 x\r\n"},
                            {5000, joined + "\r\n"},
                            {6000, "exit\r\n"}};
    setup();   // консоль выполняется целиком в setup - проверки по итогу
    runTo(20000);
    bool same = true;   // во флеш - из склеенной строки (последней верной)
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        same = same && g_flash.trig[g] == 70u - g && g_flash.hold[g] == 40u - g;
    }
    CHECK(same, "пороги сохранены (ворота 0: %u / %u, ворота 15: %u / %u)", g_flash.trig[0], g_flash.hold[0],
          g_flash.trig[15], g_flash.hold[15]);
    CHECK(countIn(g_consoleOut, "пороги ворот: 1\n") == SENSOR_GATES, "построчно: строк по одним воротам %d (ждали %u)",
          countIn(g_consoleOut, "пороги ворот: 1\n"), SENSOR_GATES);
    CHECK(countIn(g_consoleOut, "пороги ворот: 16\n") == 1, "одной строкой (%u символов): 16 ворот",
          (unsigned)joined.size());
    CHECK(countIn(g_consoleOut, "неверная команда") == 3, "ворота 16, обрезанная, лишний символ - неверные (%d)",
          countIn(g_consoleOut, "неверная команда"));
    CHECK(g_sensor.trig[3] == 67 && g_sensor.hold[15] == 25, "в датчике: ворота 3 trig %u, ворота 15 hold %u",
          g_sensor.trig[3], g_sensor.hold[15]);
}

// Консоль: sens W V - параметр сразу в датчик (принят / отказ), датчик после - как в обычной работе.
static void scConsoleSens() {
    g_world.consoleJumper = true;
    g_world.consoleInput = {{500, "sens 2 80\r\n"}, {1500, "sens 2 7\r\n"}, {2500, "exit\r\n"}};
    setup();   // консоль выполняется целиком в setup - проверки по итогу
    runTo(5000);
    CHECK(countIn(g_consoleOut, "слово 2 = 80 - принято") == 1, "ответ: принято");
    CHECK(countIn(g_consoleOut, "слово 2 = 7 - отказ") == 1, "sens 2 7 - отказ датчика");
    CHECK(g_sensor.statusRateX10 == 80, "после отказа - прежнее %u", g_sensor.statusRateX10);
    checkSensorIdle("после sens");
}

// Консоль: монитор с энергиями ворот (банка на воротах 2). eol - конец строки терминала.
static void consoleMon(const char* eol) {
    g_sensor.jarGate = 2;
    g_world.consoleJumper = true;
    g_world.consoleInput = {{500, std::string("mon") + eol}, {6000, "q"}, {7000, std::string("exit") + eol}};
    setup();
    runTo(9000);
    const int lines = countIn(g_consoleOut, "энергия макс. ворота 2:");
    CHECK(lines >= 4, "строк монитора с энергией ворот 2: %d (ждали ~5)", lines);
    checkSensorIdle("после монитора");
}
static void scConsoleMon() { consoleMon("\n"); }
static void scConsoleMonCrLf() { consoleMon("\r\n"); }

// Консоль: диагностика датчика.
static void scConsoleSensor() {
    g_world.consoleJumper = true;
    g_world.consoleInput = {{500, "sensor\n"}, {8000, "exit\n"}};
    setup();
    runTo(10000);
    const char* expect[] = {"ответ на команды: есть", "прошивка датчика v1.1.1_32768.0",
                            "Device ID: 0x4150344C (L4PA)", "серийный номер: 12345678",
                            "дальние ворота 12, ближние ворота 0, без человека 40 с",
                            "отчёт состояния 4.0 Гц, отчёт расстояния 0.5 Гц, отклик 5 (обычный)",
                            "пороги в датчике, ворота: trig / hold"};
    for (size_t i = 0; i < sizeof(expect) / sizeof(expect[0]); i++) {
        CHECK(countIn(g_consoleOut, expect[i]) == 1, "sensor: «%s»", expect[i]);
    }
    checkSensorIdle("после диагностики");
}

// Настройка задержки кнопкой: 5 с - вход, короткое +10, длинное -10, 30 с - выход с записью в датчик.
static void scSetDelay() {
    setup();
    runTo(5000);
    press(6000);
    runTo(simNowMs() + 2000);
    press(300);
    runTo(simNowMs() + 1000);
    press(300);
    runTo(simNowMs() + 1000);
    press(1500);
    runTo(simNowMs() + DELAY_EXIT_MS + 2000);
    CHECK(g_flash.offDelay == OFF_DELAY_S + 10 && g_saves == 1, "задержка сохранена %u с (ждали %u)", g_flash.offDelay,
          OFF_DELAY_S + 10);
    CHECK(g_sensor.offDelayS == OFF_DELAY_S + 10u, "задержка записана в датчик (%u)", g_sensor.offDelayS);
    checkSensorIdle("после настройки задержки");
}

#ifndef NIGHT_LIGHT
// Обрыв D1: индикатор «обрыв стабилитрона», лампа работает.
static void scSupplyFault() {
    g_world.supplyMv = 4700;
    setup();
    runTo(3000);
    CHECK(ledAt(simNowMs()) == (int)Led::SupplyFault, "индикатор обрыва D1 (%s)", ledName(ledAt(simNowMs())));
    g_sensor.personGate = 3;
    runTo(5000);
    CHECK(lampAt(simNowMs()) > 0, "лампа работает");
}
#else

// Ночник: перегрев - яркость x2/3 раз в 2 мин до уровня 0, остыл - обратно.
static void scNightOverheat() {
    presetCalib();
    g_sensor.personGate = 3;
    setup();
    runTo(20000);
    simSay("перегрев: 70 °C");
    g_world.tempC = 70;
    runTo(495000);
    simSay("остыл: 40 °C");
    g_world.tempC = 40;
    runTo(1000000);
    std::string seq;
    int minPct = 100;
    for (size_t i = 0; i < g_lampLog.size(); i++) {
        char s[8];
        snprintf(s, sizeof(s), " %d", g_lampLog[i].value);
        seq += s;
        minPct = g_lampLog[i].value < minPct ? g_lampLog[i].value : minPct;
    }
    CHECK(seq == " 20 13 8 5 7 10 15 20", "яркость по шагам:%s (ждали 20 13 8 5 7 10 15 20)", seq.c_str());
    for (size_t i = 2; i < g_lampLog.size(); i++) {
        CHECK(g_lampLog[i].ms - g_lampLog[i - 1].ms >= NIGHT_TEMP_STEP_MS, "шаг %u не чаще %u с (%.1f с)",
              (unsigned)i, NIGHT_TEMP_STEP_MS / 1000, (g_lampLog[i].ms - g_lampLog[i - 1].ms) / 1e3);
    }
}
#endif

#ifndef NIGHT_LIGHT
// Лампа 230 В, никого нет, светло: короткое нажатие - вкл, как по датчику (свет не проверяется); второе, пока горит, -
// ничего не меняет; без присутствия гаснет через minon от нажатия.
static void scButtonOn() {
    g_world.ambient = 2300;
    setup();
    runTo(20000);
    press(300);
    runTo(simNowMs() + 500);
    CHECK(lampAt(simNowMs()) == LAMP_ON_PCT && g_saves == 0, "нажатие при выкл: горит %d %% (ждали %d), записей %d",
          lampAt(simNowMs()), LAMP_ON_PCT, g_saves);
    press(300);
    const uint32_t last = simNowMs();
    runTo(last + 500);
    CHECK(lampAt(simNowMs()) == LAMP_ON_PCT && g_saves == 0, "нажатие, пока горит: горит %d %%, записей %d",
          lampAt(simNowMs()), g_saves);
    runTo(last + MIN_ON_DEFAULT_S * 1000 + 2000);
    const uint32_t off = firstLamp(false, last);
    CHECK(off != NONE && near(off, last + MIN_ON_DEFAULT_S * 1000, 200), "погас через %.1f с после нажатия (ждали %u)",
          (off - last) / 1e3, MIN_ON_DEFAULT_S);
}
#else
// Пороги дежурки как после калибровки presetCalib со светом дежурки +100 (lampLight 2000 при 5 %).
static void presetStandby(uint8_t level) {
    presetCalib();
    g_flash.standbyLevel = level;
    g_flash.standbyOnThreshold = 800;
    for (uint8_t i = 0; i < STANDBY_LEVEL_COUNT; i++) {   // свет дежурки +20 / +60 / +100
        g_flash.standbyOffThreshold[i] = 1000 + 2000 * STANDBY_LEVELS[i] / 100;
        g_flash.lightThresholdStandby[i] = 1300 + 2000 * STANDBY_LEVELS[i] / 100;
    }
    g_world.lampLight = 2000;
}

// Темно, никого: короткие нажатия - дежурка 1 -> 3 -> 5 % -> выкл по кругу, каждое сохранено.
static void scStandbyButton() {
    presetStandby(0);
    setup();
    runTo(10000);
    CHECK(lampAt(simNowMs()) == 0, "дежурка выкл - темно, ночник не горит");
    int seen[STANDBY_LEVEL_COUNT + 1];   // яркость после каждого нажатия
    for (uint8_t i = 0; i <= STANDBY_LEVEL_COUNT; i++) {
        press(300);
        runTo(simNowMs() + 1000);
        seen[i] = lampAt(simNowMs());
    }
    CHECK(seen[0] == STANDBY_LEVELS[0] && seen[1] == STANDBY_LEVELS[1] && seen[2] == STANDBY_LEVELS[2] && seen[3] == 0,
          "по кругу: %d %d %d %d %%", seen[0], seen[1], seen[2], seen[3]);
    CHECK(g_saves == STANDBY_LEVEL_COUNT + 1 && g_flash.standbyLevel == 0, "сохранено %d раз, уровень %u", g_saves,
          g_flash.standbyLevel);
}

// Дежурка 1 % по фоторезистору: светло - не горит; темнее sbon - горит; свет дежурки не гасит её (ниже sboff);
// светлее sboff - гаснет. Светло: нажатие - новый уровень показывается 3 с.
static void scStandbyLight() {
    presetStandby(1);
    g_world.ambient = 2300;
    setup();
    runTo(10000);
    CHECK(lampAt(simNowMs()) == 0, "светло - дежурка не горит");
    simSay("стемнело: 700");
    g_world.ambient = 700;
    uint32_t t = simNowMs();
    const uint32_t on = runUntil([] { return lampAt(simNowMs()) > 0; }, STANDBY_CHECK_MS + 1000);
    CHECK(on != NONE && on - t <= STANDBY_CHECK_MS + 100 && lampAt(simNowMs()) == STANDBY_LEVELS[0],
          "дежурка вкл за %.1f с (замер раз в %.0f с): %d %%", (on - t) / 1e3, STANDBY_CHECK_MS / 1e3,
          lampAt(simNowMs()));
    simSay("светлее, но ниже sboff уровня 1 (1020): 980 + свет дежурки 20");
    g_world.ambient = 980;
    runTo(simNowMs() + 3 * STANDBY_CHECK_MS);
    CHECK(lampAt(simNowMs()) == STANDBY_LEVELS[0], "дежурка горит (гистерезис)");
    simSay("утро: 1100");
    g_world.ambient = 1100;
    t = simNowMs();
    const uint32_t off = runUntil([] { return lampAt(simNowMs()) == 0; }, STANDBY_CHECK_MS + 1000);
    CHECK(off != NONE && off - t <= STANDBY_CHECK_MS + 100, "утро: дежурка погасла за %.1f с", (off - t) / 1e3);
    g_world.ambient = 2300;
    runTo(simNowMs() + 1000);
    press(300);   // уровень 1 -> 2: светло - показ
    t = simNowMs();
    runTo(t + 500);
    CHECK(lampAt(simNowMs()) == STANDBY_LEVELS[1], "светло: показ уровня %d %%", lampAt(simNowMs()));
    runTo(t + NIGHT_PREVIEW_MS + 500);
    CHECK(lampAt(simNowMs()) == 0, "показ кончился - погашена");
}

// Дежурка горит: человек - основной свет на уровне ночника (порог lightsb), ушёл - снова дежурка; нажатие, пока
// основной горит, - следующий уровень ночника (не дежурка).
static void scStandbyPresence() {
    presetStandby(3);
    g_flash.standbyOffThreshold[2] = 2000;   // дежурку 5 % свет не гасит до 2000
    g_world.ambient = 700;
    setup();
    runTo(10000);
    CHECK(lampAt(simNowMs()) == STANDBY_LEVELS[2], "темно - дежурка %d %%", lampAt(simNowMs()));
    simSay("светлее 1250: с дежуркой 1350 - выше light 1300, ниже lightsb 1400");
    g_world.ambient = 1250;
    runTo(simNowMs() + STANDBY_CHECK_MS + 1000);
    simSay("человек вошёл");
    g_sensor.personGate = 2;
    const uint32_t in = simNowMs();
    const uint32_t on = runUntil([] { return lampAt(simNowMs()) == LAMP_ON_PCT; }, 3000);
    CHECK(on != NONE, "основной свет вкл через %.1f с (порог lightsb при дежурке)", (on - in) / 1e3);
    press(300);
    runTo(simNowMs() + 500);
    CHECK(lampAt(simNowMs()) == NIGHT_LEVELS[NIGHT_DEFAULT_LEVEL + 1] && g_flash.standbyLevel == 3,
          "нажатие при основном: уровень ночника %d %%, дежурка не тронута (%u)", lampAt(simNowMs()),
          g_flash.standbyLevel);
    simSay("человек вышел");
    g_sensor.personGate = -1;
    const uint32_t left = simNowMs();
    runTo(left + (OFF_DELAY_S + MIN_ON_DEFAULT_S) * 1000 + 2000);
    CHECK(lampAt(simNowMs()) == STANDBY_LEVELS[2], "основной погас - снова дежурка %d %%", lampAt(simNowMs()));
}
#endif

struct Scenario {
    const char* name;
    void (*run)();
};
static const Scenario SCENARIOS[] = {
    {"boot_default", scBootDefault},
    {"boot_bright", scBootBright},
    {"calib_from_v6", scCalibFromV6},
    {"calib_from_v7", scCalibFromV7},
    {"calib_from_v8", scCalibFromV8},
    {"calib_ok", scCalibOk},
    {"calib_late_entry", scCalibLateEntry},
    {"calib_slow_sensor", scCalibSlowSensor},
    {"calib_dead_sensor", scCalibDeadSensor},
    {"calib_no_person", scCalibNoPerson},
    {"stuck_jar", scStuckJar},
    {"stuck_false_start", scStuckFalseStart},
    {"stuck_light_change", scStuckLightChange},
    {"stuck_then_calib", scStuckThenCalib},
    {"console_settings", scConsoleSettings},
    {"console_thresholds", scConsoleThresholds},
    {"console_sens", scConsoleSens},
    {"console_mon", scConsoleMon},
    {"console_mon_crlf", scConsoleMonCrLf},
    {"console_sensor", scConsoleSensor},
    {"set_delay", scSetDelay},
#ifndef NIGHT_LIGHT
    {"button_on", scButtonOn},
#else
    {"standby_button", scStandbyButton},
    {"standby_light", scStandbyLight},
    {"standby_presence", scStandbyPresence},
#endif
#ifndef NIGHT_LIGHT
    {"supply_fault", scSupplyFault},
#else
    {"night_overheat", scNightOverheat},
    {"console_night", scConsoleNight},
    {"set_pwm", scSetPwm},
#endif
};

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOFBF, 1 << 16);
    if (argc < 2 || !strcmp(argv[1], "list")) {
        for (size_t i = 0; i < sizeof(SCENARIOS) / sizeof(SCENARIOS[0]); i++) {
            fprintf(stdout, "%s\n", SCENARIOS[i].name);
        }
        return 0;
    }
    for (size_t i = 0; i < sizeof(SCENARIOS) / sizeof(SCENARIOS[0]); i++) {
        if (!strcmp(argv[1], SCENARIOS[i].name)) {
            SCENARIOS[i].run();
            CHECK(g_sensor.violations == 0, "нарушений обмена с датчиком нет (%d)", g_sensor.violations);
            CHECK(g_deepDrops == 0, "во сне с остановкой тактов при открытом UART байты не терялись (%d)", g_deepDrops);
            fprintf(stdout, "ИТОГ: %s, проверок %d, провалено %d\n", s_fails ? "ПРОВАЛ" : "ок", s_checks, s_fails);
            return s_fails ? 1 : 0;
        }
    }
    fprintf(stdout, "нет сценария %s\n", argv[1]);
    return 2;
}
