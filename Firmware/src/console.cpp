// Консоль настройки по USB: открывается только при включении с перемычкой (boardConsoleOpen), до обычной работы - поэтому
// ожидание ввода через delay() (этап setup). Выход - exit или CONSOLE_IDLE_MS без команд.
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "console.h"
#include "ld2410s.h"

constexpr int KEY_BACKSPACE = '\b';   // стирание символа (Ctrl+H)
constexpr int KEY_DEL       = 0x7F;   // стирание символа - так шлёт Backspace большинство терминалов
// Общие параметры датчика для sensor (табл. 2-2): дальние ворота, ближние ворота, время без человека, частота отчёта
// состояния, частота отчёта расстояния (обе - Гц x 10: в примере п. 2.2.8 0,5 Гц = 5), скорость отклика
constexpr uint16_t DIAG_PARAM_WORDS[] = {0x05, 0x0A, 0x06, 0x02, 0x0C, 0x0B};
constexpr uint8_t  DIAG_PARAM_COUNT   = sizeof(DIAG_PARAM_WORDS) / sizeof(DIAG_PARAM_WORDS[0]);   // их число
constexpr uint32_t SPEED_NORMAL       = 5;    // скорость отклика «обычная» (табл. 2-2)
constexpr uint32_t SPEED_FAST         = 10;   // скорость отклика «быстрая» (табл. 2-2)
constexpr uint8_t  DIAG_VER_MAX       = 16;   // ответ о версии прошивки датчика: показать байт
constexpr uint8_t  DIAG_VER_NUMS      = 12;   // ответ о версии: статус + 5 слов, байт (перехват программы HLK)
constexpr uint8_t  DIAG_INFO_LEN      = 6;    // ответ Obtain firmware info: статус + 4 байта

static void help(Stream& s) {
    s.printf("Команды (значение сохраняется сразу):\n"
             "  show        - показать настройки\n"
             "  delay N     - задержка выключения, с: %u..%u, шаг %u\n"
             "  minon N     - свет горит не меньше N с от последнего импульса OT2 (против дребезга): 0..65535\n"
             "  light N     - АЦП: порог света, отсчёты 0..%lu\n"
             "  ldr 0|1|2   - фоторезистор: 0 - нет (свет не проверяется, всегда «темно»); 1 - больше света -\n"
             "                больший отсчёт АЦП (темно - малый); 2 - больше света - меньший отсчёт (темно - большой)\n"
             "  far N       - дальние ворота 1..%u (дальность, ворота около 0,7 м); 0 - не писать в датчик\n"
             "  trig G N    - порог срабатывания ворот G (0..%u)\n"
             "  hold G N    - порог удержания ворот G\n",
             OFF_DELAY_MIN_S, OFF_DELAY_MAX_S, OFF_DELAY_STEP_S, (unsigned long)ADC_FULL, SENSOR_FAR_MAX,
             SENSOR_GATES - 1);
#ifdef NIGHT_LIGHT
    s.printf("  level N     - выбранный уровень яркости ночника 0..%u\n"
             "  pct L P     - яркость уровня L, %%: 1..%lu (уровень 0 - минимум при перегреве)\n"
             "  tmax N      - порог перегрева, °C: %d..%d (выше - яркость на 1/3 меньше раз в %lu с)\n"
             "  on P        - включить ночник с яркостью P, %%: 0..%lu (0 - выкл; проверка, не сохраняется)\n",
             NIGHT_LEVEL_COUNT - 1, (unsigned long)NIGHT_PWM_RANGE, NIGHT_TMAX_MIN_C, NIGHT_TMAX_MAX_C,
             (unsigned long)(NIGHT_TEMP_STEP_MS / 1000), (unsigned long)NIGHT_PWM_RANGE);
#endif
    s.printf("  sensor      - диагностика датчика: отчёты по UART, ответ на команды, пороги в датчике\n"
             "  mon         - монитор: OT2, отчёт датчика, свет, кнопка BOOT; выход - любая клавиша\n"
             "  calib       - выйти и запустить калибровку (как удержание BOOT %lu с)\n"
             "  reset       - настройки по умолчанию (калибровка стирается)\n"
             "  exit        - выйти и начать работу (сам - через %lu с без команд)\n",
             (unsigned long)(CAL_PRESS_MS / 1000), (unsigned long)(CONSOLE_IDLE_MS / 1000));
}

static void show(Stream& s, const Calib& c) {
    s.printf("Версия %s, калибровка датчика: %s\n", FW_VERSION,
             sensorCalibrated(c) ? "есть" : "нет (заводские пороги)");
    s.printf("delay %u с, minon %u с, far %u", c.offDelay, c.minOnS, c.farGate);
#ifdef NIGHT_LIGHT
    s.printf(", level %u, pct", c.nightLevel);
    for (uint8_t l = 0; l < NIGHT_LEVEL_COUNT; l++) {
        s.printf(" %u", c.nightPct[l]);
    }
    s.printf(" %%\ntmax %d °C, температура сейчас %d °C", c.tempMax, boardTempC() + NIGHT_TEMP_OFFSET_C);
#endif
    if (c.ldrInputMode == LDR_NONE) {
        s.printf("\nldr 0: фоторезистора нет, свет не проверяется\n");
    } else {
        s.printf("\nАЦП: свет сейчас %u, light %u, ldr %u\n", boardLight(), c.lightThreshold, c.ldrInputMode);
    }
    s.printf("ворота: trig / hold\n");
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        s.printf("%2u: %lu / %lu\n", g, (unsigned long)c.trig[g], (unsigned long)c.hold[g]);
    }
}

// Диагностика датчика: приём отчётов DIAG_REPORTS_MS (линия OT1 -> GP1), ответ на команды (GP0 -> RX), пороги.
static void sensorDiag(Stream& s) {
    Stream& u = boardSensorBegin();
    uint32_t bytes = 0;          // принято байт за DIAG_RAW_MS (без разбора кадров)
    uint8_t first[DIAG_RAW_SHOW];   // первые принятые байты
    const uint32_t rawStart = millis();
    while (millis() - rawStart < DIAG_RAW_MS) {
        for (int b = u.read(); b >= 0; b = u.read()) {
            if (bytes < DIAG_RAW_SHOW) {
                first[bytes] = (uint8_t)b;
            }
            bytes++;
        }
        delay(USB_POLL_MS);
    }
    s.printf("байт по UART за %lu мс: %lu", (unsigned long)DIAG_RAW_MS, (unsigned long)bytes);
    for (uint32_t i = 0; i < bytes && i < DIAG_RAW_SHOW; i++) {
        s.printf(" %02X", first[i]);
    }
    s.printf("\n");
    ldBegin(u);
    LdReport r;                  // последний отчёт
    uint16_t reports = 0;        // принято отчётов
    const uint32_t start = millis();
    while (millis() - start < DIAG_REPORTS_MS) {
        while (ldPoll(r)) {
            reports++;
        }
        delay(USB_POLL_MS);
    }
    s.printf("OT2 (GP2): %u\nотчётов за %lu мс: %u", boardPresence(), (unsigned long)DIAG_REPORTS_MS, reports);
    if (reports) {
        s.printf(", последний: %s, %u см", r.present ? "человек" : "никого", r.distanceCm);
    }
    uint32_t trig[SENSOR_GATES];   // пороги срабатывания из датчика
    uint32_t hold[SENSOR_GATES];   // пороги удержания из датчика
    uint16_t proto = 0, bufSize = 0;   // версия протокола и размер буфера (ACK входа в режим команд)
    uint8_t ver[DIAG_VER_MAX];     // ответ датчика на чтение версии прошивки, байты
    char sn[LD_SN_LEN + 1];        // серийный номер
    uint32_t par[DIAG_PARAM_COUNT];   // общие параметры (DIAG_PARAMS)
    const bool cfg = ldEnableConfig(&proto, &bufSize);
    const uint8_t verLen = cfg ? ldQuery(LD_CMD_VERSION, ver, sizeof(ver)) : 0;   // байт в ответе о версии
    uint8_t info[3][DIAG_INFO_LEN] = {};   // ответы Obtain firmware info: 0x0060, 0x0061, 0x0065 (Device ID)
    uint8_t infoLen[3] = {};          // их длины, байт
    const uint16_t infoCmd[3] = {LD_CMD_FW_INFO_60, LD_CMD_FW_INFO_61, LD_CMD_DEVICE_ID};   // команды
    for (uint8_t i = 0; cfg && i < 3; i++) {
        infoLen[i] = ldQuery(infoCmd[i], info[i], DIAG_INFO_LEN);
    }
    const bool snOk = cfg && ldReadSerial(sn);
    const bool parOk = cfg && ldReadParams(DIAG_PARAM_WORDS, DIAG_PARAM_COUNT, par);
    const bool thr = cfg && ldReadThresholds(LD_CMD_READ_TRIG, trig) && ldReadThresholds(LD_CMD_READ_HOLD, hold);
    ldEndConfig();
    boardSensorEnd();
    s.printf("\nответ на команды: %s\n", cfg ? "есть" : "НЕТ");
    if (cfg) {
        s.printf("протокол %u, буфер %u байт\n", proto, bufSize);
        if (verLen >= DIAG_VER_NUMS) {   // статус(2), 2 слова, 3 слова - у программы HLK «v1.1.1_32768.0»
            s.printf("прошивка датчика v%u.%u.%u_%u.%u\n", ver[6] | (ver[7] << 8), ver[8] | (ver[9] << 8),
                     ver[10] | (ver[11] << 8), ver[2] | (ver[3] << 8), ver[4] | (ver[5] << 8));
        } else {
            s.printf("прошивка датчика: нет ответа\n");
        }
        // Obtain firmware info: статус(2) + 4 байта; у 0x0065 - символы
        s.printf("firmware info: 0x0060 = %lu, 0x0061 = %lu\n"
                 "Device ID: 0x%02X%02X%02X%02X (%.4s)\n",
                 infoLen[0] >= DIAG_INFO_LEN ? (unsigned long)info[0][2] | ((unsigned long)info[0][3] << 8) |
                     ((unsigned long)info[0][4] << 16) | ((unsigned long)info[0][5] << 24) : 0UL,
                 infoLen[1] >= DIAG_INFO_LEN ? (unsigned long)info[1][2] | ((unsigned long)info[1][3] << 8) |
                     ((unsigned long)info[1][4] << 16) | ((unsigned long)info[1][5] << 24) : 0UL,
                 info[2][5], info[2][4], info[2][3], info[2][2],   // младший байт вперёд, как у программы HLK
                 infoLen[2] >= DIAG_INFO_LEN ? (const char*)&info[2][2] : "?");
        s.printf("серийный номер: %s\n", snOk ? sn : "нет ответа");
    }
    if (parOk) {
        s.printf("дальние ворота %lu, ближние ворота %lu, без человека %lu с\n"
                 "отчёт состояния %lu.%lu Гц, отчёт расстояния %lu.%lu Гц, отклик %lu (%s)\n",
                 (unsigned long)par[0], (unsigned long)par[1], (unsigned long)par[2],
                 (unsigned long)(par[3] / 10), (unsigned long)(par[3] % 10),
                 (unsigned long)(par[4] / 10), (unsigned long)(par[4] % 10), (unsigned long)par[5],
                 par[5] == SPEED_FAST ? "быстрый" : par[5] == SPEED_NORMAL ? "обычный" : "?");
    } else if (cfg) {
        s.printf("общие параметры: нет ответа\n");
    }
    if (thr) {
        s.printf("пороги в датчике, ворота: trig / hold\n");
        for (uint8_t g = 0; g < SENSOR_GATES; g++) {
            s.printf("%2u: %lu / %lu\n", g, (unsigned long)trig[g], (unsigned long)hold[g]);
        }
    }
}

// Монитор до любой клавиши: смена OT2 и кнопки - сразу, строка состояния - раз в MONITOR_PRINT_MS.
static void monitor(Stream& s) {
    ldBegin(boardSensorBegin());
    LdReport r;                  // последний отчёт датчика
    uint16_t reports = 0;        // отчётов с последней строки состояния
    bool ot2 = boardPresence();  // OT2 в прошлом опросе
    bool btn = boardButton();    // кнопка BOOT в прошлом опросе
    uint32_t printMs = millis() - MONITOR_PRINT_MS;   // время последней строки состояния (millis)
    s.printf("монитор, выход - любая клавиша\n");
    while (s.read() < 0) {
        const uint32_t now = millis();
        while (ldPoll(r)) {
            reports++;
        }
        if (boardPresence() != ot2 || boardButton() != btn) {
            btn = boardButton();
            ot2 = boardPresence();
            s.printf("%lu мс: OT2 %u, BOOT %u\n", (unsigned long)now, ot2, btn);
        }
        if (now - printMs >= MONITOR_PRINT_MS) {
            printMs = now;
            s.printf("OT2 %u, BOOT %u, свет %u, отчётов %u", ot2, btn, boardLight(), reports);
            if (reports) {
                s.printf(" (%s, %u см)", r.present ? "человек" : "никого", r.distanceCm);
            }
            s.printf("\n");
            reports = 0;
        }
        delay(USB_POLL_MS);
    }
    boardSensorEnd();
}

// Выполнить строку команды; false - выйти из консоли (calib = true - и запустить калибровку).
static bool execute(Stream& s, Calib& c, const char* line, bool& calib) {
    char cmd[8] = "";            // слово команды
    unsigned long a = 0, b = 0;  // числовые аргументы
    const int n = sscanf(line, "%7s %lu %lu", cmd, &a, &b);
    if (!strcmp(cmd, "exit")) {
        return false;
    }
    if (n == 1 && !strcmp(cmd, "calib")) {
        calibPlan(s);
        s.printf("Enter - начать, другая клавиша - отмена\n");
        delay(CONSOLE_FLUSH_MS);   // остаток конца строки команды (\r\n) - не принять за Enter
        while (s.read() >= 0) {
        }
        int ch = -1;   // ответ: клавиша
        const uint32_t waitMs = millis();   // начало ожидания (millis)
        while ((ch = s.read()) < 0 && millis() - waitMs < CONSOLE_IDLE_MS) {
            delay(USB_POLL_MS);
        }
        if (ch == '\r' || ch == '\n') {
            calib = true;
            return false;
        }
        s.printf("калибровка отменена\n");
        return true;
    }
    if (!strcmp(cmd, "show")) {
        show(s, c);
        return true;
    }
    if (!strcmp(cmd, "help")) {
        help(s);
        return true;
    }
    if (!strcmp(cmd, "sensor")) {
        sensorDiag(s);
        return true;
    }
    if (!strcmp(cmd, "mon")) {
        monitor(s);
        return true;
    }
#ifdef NIGHT_LIGHT
    if (n == 2 && !strcmp(cmd, "on") && a <= NIGHT_PWM_RANGE) {
        boardConsoleLamp((uint8_t)a);
        s.printf("ночник %lu %%\n", a);
        return true;
    }
#endif
    bool ok = false;   // команда и значение приняты
    if (n == 1 && !strcmp(cmd, "reset")) {
        calibDefaults(c);
        ok = true;
    } else if (n == 2 && !strcmp(cmd, "delay") && a >= OFF_DELAY_MIN_S && a <= OFF_DELAY_MAX_S &&
               a % OFF_DELAY_STEP_S == 0) {
        c.offDelay = (uint8_t)a;
        ok = true;
    } else if (n == 2 && !strcmp(cmd, "minon") && a <= UINT16_MAX) {
        c.minOnS = (uint16_t)a;
        ok = true;
    } else if (n == 2 && !strcmp(cmd, "ldr") && a <= LDR_DARK_HIGH) {
        c.ldrInputMode = (LdrInputMode)a;
        ok = true;
    } else if (n == 2 && !strcmp(cmd, "light") && a <= ADC_FULL) {
        c.lightThreshold = (uint16_t)a;
        ok = true;
    } else if (n == 2 && !strcmp(cmd, "far") && a <= SENSOR_FAR_MAX) {
        c.farGate = (uint8_t)a;
        ok = true;
    } else if (n == 3 && (!strcmp(cmd, "trig") || !strcmp(cmd, "hold")) && a < SENSOR_GATES) {
        (cmd[0] == 't' ? c.trig : c.hold)[a] = b;
        ok = true;
#ifdef NIGHT_LIGHT
    } else if (n == 2 && !strcmp(cmd, "level") && a < NIGHT_LEVEL_COUNT) {
        c.nightLevel = (uint8_t)a;
        ok = true;
    } else if (n == 3 && !strcmp(cmd, "pct") && a < NIGHT_LEVEL_COUNT && b >= 1 && b <= NIGHT_PWM_RANGE) {
        c.nightPct[a] = (uint8_t)b;
        ok = true;
    } else if (n == 2 && !strcmp(cmd, "tmax") && a >= NIGHT_TMAX_MIN_C && a <= NIGHT_TMAX_MAX_C) {
        c.tempMax = (int8_t)a;
        ok = true;
#endif
    }
    if (ok) {
        boardSave(c);
        s.printf("сохранено\n");
    } else {
        s.printf("неверная команда или значение, help - список\n");
    }
    return true;
}

bool consoleRun(Stream& s, Calib& c) {
    bool calib = false;          // введена calib - после выхода запустить калибровку
    char line[CONSOLE_LINE];     // вводимая строка
    uint8_t len = 0;             // символов в строке
    bool shown = false;          // приветствие выведено в открытый монитор порта
    uint32_t lastMs = millis();  // включение или последняя команда, мс (millis)
    while (millis() - lastMs < CONSOLE_IDLE_MS) {
        if (!boardConsoleConnected()) {   // монитор порта не открыт или закрыт - ждать до срока
            shown = false;
            len = 0;
            delay(USB_POLL_MS);
            continue;
        }
        if (!shown) {
            s.printf("\nАвтолампа %s: консоль настройки\n", FW_VERSION);
            help(s);
            show(s, c);
            s.printf("> ");
            shown = true;
        }
        const int ch = s.read();
        if (ch < 0) {
            delay(USB_POLL_MS);
        } else if (ch == '\r' || ch == '\n') {
            if (len == 0) {
                continue;   // вторая половина \r\n или пустая строка
            }
            line[len] = '\0';
            len = 0;
            s.printf("\n");
            if (!execute(s, c, line, calib)) {
                break;
            }
            lastMs = millis();   // после команды: монитор может идти дольше CONSOLE_IDLE_MS
            s.printf("> ");
        } else if ((ch == KEY_BACKSPACE || ch == KEY_DEL) && len > 0) {
            len--;
            s.printf("\b \b");
        } else if (ch >= ' ' && len < CONSOLE_LINE - 1) {
            line[len++] = (char)ch;
            s.write((uint8_t)ch);   // эхо: монитор порта сам не показывает ввод
        }
    }
    if (boardConsoleConnected()) {
        s.printf(calib ? "Выход из консоли, калибровка: %lu с выйти из комнаты\n" : "Выход из консоли, работа\n",
                 (unsigned long)CAL_LEAVE_S);
    }
    return calib;
}
