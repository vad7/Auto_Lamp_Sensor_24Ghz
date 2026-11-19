// Симулятор на ПК: часы, заглушка Arduino, плата (board.h) и модель датчика HLK-LD2410S.
#include "sim.h"
#include <algorithm>

uint64_t g_simUs = 0;
bool     g_deepSleep = false;
SensorModel g_sensor;
World    g_world;
std::vector<Change> g_lampLog;
std::vector<Change> g_ledLog;
Calib    g_flash;
bool     g_flashValid = false;
int      g_saves = 0;
uint32_t g_lampPwmHz = 0;
int      g_deepDrops = 0;
std::string g_consoleOut;
std::string g_serialOut;
SimSerial Serial;

static int      s_lampPct = 0;      // яркость лампы сейчас, %
static int      s_led = 0;          // индикатор сейчас (Led)
static uint32_t s_lightRnd = 777;   // помеха фоторезистора (детерминированная)

// ---------------------------------------------------------------- часы, вывод

uint32_t simNowMs() { return (uint32_t)(g_simUs / 1000); }

void simAdvanceUs(uint64_t us) {
    g_simUs += us;
    g_sensor.update(g_simUs);
}

uint32_t millis() {
    simAdvanceUs(SIM_CALL_COST_US);
    return simNowMs();
}

void delay(uint32_t ms) { simAdvanceUs((uint64_t)ms * 1000); }

// Строки вывода с отметкой времени: копим до \n.
static void lineOut(std::string& pending, const char* tag, uint8_t b) {
    if (b == '\n') {
        fprintf(stdout, "[%10.3f] %s%s\n", g_simUs / 1e6, tag, pending.c_str());
        pending.clear();
    } else if (b != '\r') {
        pending += (char)b;
    }
}

void simSay(const char* fmt, ...) {
    char text[256];
    va_list a;
    va_start(a, fmt);
    vsnprintf(text, sizeof(text), fmt, a);
    va_end(a);
    fprintf(stdout, "[%10.3f] *** %s\n", g_simUs / 1e6, text);
}

size_t Print::printf(const char* fmt, ...) {
    char buf[1024];
    va_list a;
    va_start(a, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, a);
    va_end(a);
    if (n < 0) {
        return 0;
    }
    if (n >= (int)sizeof(buf)) {
        n = sizeof(buf) - 1;
    }
    return write((const uint8_t*)buf, (size_t)n);
}

static std::string s_serialLine;   // недописанная строка Serial
size_t SimSerial::write(uint8_t b) {
    simAdvanceUs(SIM_SERIAL_BYTE_US);
    g_serialOut += (char)b;
    lineOut(s_serialLine, "", b);
    return 1;
}

const char* ledName(int led) {
    static const char* NAMES[] = {"Off", "Red", "Green", "Blue", "CalLeave", "NoSensor", "SupplyFault", "StuckFlash"};
    return led >= 0 && led < (int)(sizeof(NAMES) / sizeof(NAMES[0])) ? NAMES[led] : "?";
}

// ---------------------------------------------------------------- консоль USB (ввод по сценарию)

class ScriptedConsole : public Stream {
public:
    std::vector<std::pair<uint32_t, char> > input;   // байт и время, с которого он доступен
    size_t pos = 0;
    std::string pending;
    size_t write(uint8_t b) {
        g_consoleOut += (char)b;
        lineOut(pending, "con> ", b);
        return 1;
    }
    int available() {
        int n = 0;
        for (size_t i = pos; i < input.size() && input[i].first <= simNowMs(); i++) {
            n++;
        }
        return n;
    }
    int read() {
        if (pos < input.size() && input[pos].first <= simNowMs()) {
            return (uint8_t)input[pos++].second;
        }
        return -1;
    }
};
static ScriptedConsole s_console;

// ---------------------------------------------------------------- UART датчика

class SensorUart : public Stream {
public:
    size_t write(uint8_t b) {
        if (!g_sensor.uartOpen) {
            g_sensor.violations++;
            simSay("НАРУШЕНИЕ: запись в закрытый UART датчика");
            return 0;
        }
        g_sensor.hostByte(b);
        return 1;
    }
    int available() {
        g_sensor.update(g_simUs);
        return (int)g_sensor.rx.size();
    }
    int read() {
        g_sensor.update(g_simUs);
        if (g_sensor.rx.empty()) {
            return -1;
        }
        const uint8_t b = g_sensor.rx.front();
        g_sensor.rx.pop_front();
        return b;
    }
};
static SensorUart s_sensorUart;

// ---------------------------------------------------------------- плата (board.h)

Stream* boardConsoleOpen() {
    if (!g_world.consoleJumper) {
        return nullptr;
    }
    for (size_t i = 0; i < g_world.consoleInput.size(); i++) {
        const std::string& text = g_world.consoleInput[i].second;
        for (size_t k = 0; k < text.size(); k++) {
            s_console.input.push_back(std::make_pair(g_world.consoleInput[i].first, text[k]));
        }
    }
    return &s_console;
}

bool boardConsoleConnected() { return true; }   // монитор порта открыт: сообщения калибровки идут в журнал

#ifdef NIGHT_LIGHT
void boardLampFreq(uint32_t hz) {
    g_lampPwmHz = hz;
    boardLamp(0);
}
#endif

void boardInit() {}

void boardSleep(uint32_t ms, bool deep) {
    g_deepSleep = deep;
    simAdvanceUs((uint64_t)ms * 1000);
    g_deepSleep = false;
}

bool boardButton() {
    simAdvanceUs(SIM_BUTTON_COST_US);
    return g_world.button;
}

bool boardPresence() {
    g_sensor.update(g_simUs);
    return g_sensor.ot2;
}

void boardLamp(uint8_t percent) {
    if (percent != s_lampPct) {
        s_lampPct = percent;
        g_lampLog.push_back(Change{simNowMs(), percent});
        simSay("лампа %u %%", percent);
    }
}

uint16_t boardLight() {
    simAdvanceUs(SIM_LIGHT_COST_US);
    s_lightRnd = s_lightRnd * 1103515245u + 12345u;
    const int noise = (int)((s_lightRnd >> 16) % 7) - 3;   // помеха -3..+3 отсчёта (модель)
    int v = g_world.ambient + g_world.lampLight * s_lampPct / 100 + noise;
    v = v < 0 ? 0 : v > (int)ADC_FULL ? (int)ADC_FULL : v;
    return (uint16_t)v;
}

uint16_t boardSupplyMv() { return g_world.supplyMv; }

int16_t boardTempC() { return g_world.tempC; }

void boardLed(Led led) {
    if ((int)led != s_led) {
        s_led = (int)led;
        g_ledLog.push_back(Change{simNowMs(), (int)led});
        simSay("LED %s", ledName((int)led));
    }
}

void boardLedTick(uint32_t now) { (void)now; }

Stream& boardSensorBegin() {
    g_sensor.uartOpen = true;
    g_sensor.rx.clear();
    return s_sensorUart;
}

void boardSensorEnd() {
    g_sensor.uartOpen = false;
    g_sensor.rx.clear();
}

bool boardLoad(Calib& c) {
    if (!g_flashValid) {
        memset(&c, 0xFF, sizeof(c));   // стёртая флеш
        return false;
    }
    c = g_flash;
    return c.magic == CALIB_MAGIC && c.version == CALIB_VERSION;
}

void boardSave(const Calib& c) {
    g_flash = c;
    g_flashValid = true;
    g_saves++;
    simSay("калибровка записана во флеш");
}

// ---------------------------------------------------------------- модель датчика

static void put16(std::vector<uint8_t>& v, uint16_t x) {
    v.push_back((uint8_t)x);
    v.push_back((uint8_t)(x >> 8));
}
static void put32(std::vector<uint8_t>& v, uint32_t x) {
    put16(v, (uint16_t)x);
    put16(v, (uint16_t)(x >> 16));
}
static uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t* p) { return get16(p) | ((uint32_t)get16(p + 2) << 16); }

SensorModel::SensorModel() {
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        trig[g] = 40;   // заводские пороги - модель (выше фона, ниже человека)
        hold[g] = 35;
        autoMaxDb[g] = 0;
    }
}

float SensorModel::jitter() {
    rnd = rnd * 1103515245u + 12345u;
    return (float)((rnd >> 16) % 2001) / 1000.0f - 1.0f;
}

void SensorModel::deliver(const std::vector<uint8_t>& bytes) {
    if (!uartOpen) {
        return;   // UART МК закрыт - байты никто не принимает
    }
    if (g_deepSleep) {
        g_deepDrops += (int)bytes.size();   // сон с остановкой тактов: UART не принимает
        return;
    }
    rx.insert(rx.end(), bytes.begin(), bytes.end());
}

void SensorModel::update(uint64_t now) {
    for (;;) {
        uint64_t t = nextFrameUs;
        int kind = 0;   // 0 - кадр, 1 - ACK, 2 - конец автокалибровки
        if (!acks.empty() && acks.front().dueUs < t) {
            t = acks.front().dueUs;
            kind = 1;
        }
        if (autoActive && autoEndUs < t) {
            t = autoEndUs;
            kind = 2;
        }
        if (t > now) {
            return;
        }
        if (kind == 1) {
            std::vector<uint8_t> bytes = acks.front().bytes;
            acks.erase(acks.begin());
            deliver(bytes);
        } else if (kind == 2) {
            autoActive = false;
            autoDone = true;
            for (uint8_t g = 0; g < SENSOR_GATES; g++) {   // пороги по фону скана: + 10 / + 7 дБ (модель)
                trig[g] = (uint32_t)(autoMaxDb[g] + 10.5f);
                hold[g] = (uint32_t)(autoMaxDb[g] + 7.5f);
            }
            silenceUntilUs = deadAfterAuto ? UINT64_MAX : autoEndUs + (uint64_t)silenceMs * 1000;
            simSay("датчик: автокалибровка 100 %%, порог ворот 0 = %u дБ; тишина %u мс%s", trig[0], silenceMs,
                   deadAfterAuto ? " - и дальше молчит" : "");
        } else {
            frame(t);
            nextFrameUs += SIM_FRAME_US;
        }
    }
}

void SensorModel::frame(uint64_t t) {
    frameNo++;
    const int pg = personMoving ? 1 + (int)((t / 1000 / SIM_PERSON_STEP_MS) % 4) : personGate;
    uint32_t energy[SENSOR_GATES];
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        double lin = pow(10.0, (bgDb + jitter()) / 10.0);
        if (pg >= 0 && abs(g - pg) <= 1) {
            lin += pow(10.0, (personDb - (g == pg ? 0 : 8) + jitter()) / 10.0);
        }
        if (jarGate >= 0 && abs(g - jarGate) <= 1) {
            lin += pow(10.0, (jarDb - (g == jarGate ? 0 : 10) + jitter()) / 10.0);
        }
        energy[g] = (uint32_t)lin;
        energyDb[g] = (float)(10.0 * log10(lin));
        if (autoActive && t >= autoStartUs && energyDb[g] > autoMaxDb[g]) {
            autoMaxDb[g] = energyDb[g];
        }
    }
    int seenGate = -1;   // ближние ворота выше порога (в пределах дальних ворот)
    for (uint8_t g = 0; g < SENSOR_GATES && g < farGate; g++) {
        if (energyDb[g] >= (ot2 ? hold[g] : trig[g])) {
            seenGate = g;
            break;
        }
    }
    if (seenGate >= 0) {
        lastSeenUs = t;
        ot2 = true;
        distanceCm = (uint16_t)(seenGate * SIM_GATE_CM);
    } else if (ot2 && t - lastSeenUs >= (uint64_t)offDelayS * 1000000) {
        ot2 = false;
    }
    if ((deadAfterAuto && autoDone) || t < silenceUntilUs) {
        return;
    }
    std::vector<uint8_t> f;
    if (autoActive) {   // ход автокалибровки: F4..F1, длина 3, тип 03, % (перехват)
        const uint64_t done = t > autoStartUs ? (t - autoStartUs) * 100 / (autoEndUs - autoStartUs) : 0;
        const uint8_t head[] = {0xF4, 0xF3, 0xF2, 0xF1, 0x03, 0x00, 0x03};
        f.assign(head, head + sizeof(head));
        put16(f, (uint16_t)(done > 99 ? 99 : done));
        const uint8_t tail[] = {0xF8, 0xF7, 0xF6, 0xF5};
        f.insert(f.end(), tail, tail + 4);
    } else if (standard) {   // стандартный: тип 01, состояние, расстояние, резерв, 16 энергий (перехват)
        const uint8_t head[] = {0xF4, 0xF3, 0xF2, 0xF1, 0x46, 0x00, 0x01};
        f.assign(head, head + sizeof(head));
        f.push_back(ot2 ? 2 : 0);
        put16(f, distanceCm);
        put16(f, 0);
        for (uint8_t g = 0; g < SENSOR_GATES; g++) {
            put32(f, energy[g]);
        }
        const uint8_t tail[] = {0xF8, 0xF7, 0xF6, 0xF5};
        f.insert(f.end(), tail, tail + 4);
    } else if (ot2 || frameNo % SIM_MIN_EVERY == 0) {   // минимальный: 6E, состояние, расстояние, 62
        f.push_back(0x6E);
        f.push_back(ot2 ? 2 : 0);
        put16(f, distanceCm);
        f.push_back(0x62);
    }
    if (!f.empty()) {
        deliver(f);
    }
}

void SensorModel::hostByte(uint8_t b) {
    static const uint8_t HEAD[4] = {0xFD, 0xFC, 0xFB, 0xFA};
    static const uint8_t TAIL[4] = {0x04, 0x03, 0x02, 0x01};
    if (hostBuf.size() < 4 && b != HEAD[hostBuf.size()]) {
        violations++;
        simSay("НАРУШЕНИЕ: байт 0x%02X вне кадра команды", b);
        hostBuf.clear();
        return;
    }
    hostBuf.push_back(b);
    if (hostBuf.size() < 6) {
        return;
    }
    const uint16_t len = get16(&hostBuf[4]);
    if (hostBuf.size() < 6u + len + 4u) {
        return;
    }
    if (len < 2 || memcmp(&hostBuf[6 + len], TAIL, 4) != 0) {
        violations++;
        simSay("НАРУШЕНИЕ: кадр команды без конца 04 03 02 01");
    } else {
        handle(get16(&hostBuf[6]), &hostBuf[8], (uint16_t)(len - 2));
    }
    hostBuf.clear();
}

void SensorModel::queueAck(uint64_t due, uint16_t cmd, uint16_t status, const std::vector<uint8_t>& data) {
    Ack a;
    a.dueUs = due;
    const uint8_t head[] = {0xFD, 0xFC, 0xFB, 0xFA};
    a.bytes.assign(head, head + 4);
    put16(a.bytes, (uint16_t)(4 + data.size()));
    put16(a.bytes, (uint16_t)(cmd | 0x0100));
    put16(a.bytes, status);
    a.bytes.insert(a.bytes.end(), data.begin(), data.end());
    const uint8_t tail[] = {0x04, 0x03, 0x02, 0x01};
    a.bytes.insert(a.bytes.end(), tail, tail + 4);
    std::vector<Ack>::iterator it = acks.begin();
    while (it != acks.end() && it->dueUs <= due) {
        ++it;
    }
    acks.insert(it, a);
}

void SensorModel::handle(uint16_t cmd, const uint8_t* v, uint16_t n) {
    Cmd logged = {simNowMs(), cmd, false};
    const uint64_t now = g_simUs;
    if (now < silenceUntilUs) {   // тишина после автокалибровки: не отвечает, потом сбивается (перехват)
        lag = !deadAfterAuto;
        lagCmd = cmd;
        cmdLog.push_back(logged);
        return;
    }
    if (cmd != 0x00FF && cmd != 0x00FE && !configMode) {   // модель: вне режима команд - не отвечает
        simSay("датчик: команда 0x%04X вне режима команд - без ответа", cmd);
        cmdLog.push_back(logged);
        return;
    }
    uint16_t status = 0;
    const std::vector<uint8_t> data = execute(cmd, v, n, status);
    if (lag && cmd != 0x00FE) {   // сбой: ответ - на предыдущую команду, без данных (кроме входа в режим команд)
        std::vector<uint8_t> prev;
        if (lagCmd == 0x00FF) {
            put16(prev, 3);
            put16(prev, 128);
        }
        queueAck(now + SIM_LAG_ACK_US, (uint16_t)lagCmd, 0, prev);
        simSay("датчик: сбой - на 0x%04X ответ 0x%04X", cmd, lagCmd);
        lagCmd = cmd;
        cmdLog.push_back(logged);
        return;
    }
    if (cmd == 0x00FE) {
        lag = false;   // выход из режима команд снимает сбой (перехват)
        lagCmd = -1;
    }
    logged.answered = true;
    cmdLog.push_back(logged);
    queueAck(now + SIM_ACK_US, cmd, status, data);
}

std::vector<uint8_t> SensorModel::execute(uint16_t cmd, const uint8_t* v, uint16_t n, uint16_t& status) {
    std::vector<uint8_t> d;
    switch (cmd) {
    case 0x00FF:
        configMode = true;
        put16(d, 3);     // версия протокола (перехват)
        put16(d, 128);   // буфер
        break;
    case 0x00FE:
        configMode = false;
        break;
    case 0x0000: {   // версия: 0x8000, 0, 1, 1, 1 (перехват)
        const uint16_t w[] = {0x8000, 0, 1, 1, 1};
        for (int i = 0; i < 5; i++) {
            put16(d, w[i]);
        }
        break;
    }
    case 0x0011:
        put16(d, 8);
        for (const char* p = "12345678"; *p; p++) {
            d.push_back((uint8_t)*p);
        }
        break;
    case 0x0060:
        put32(d, 2);
        break;
    case 0x0061:
        put32(d, 1);
        break;
    case 0x0065:
        for (const char* p = "L4PA"; *p; p++) {
            d.push_back((uint8_t)*p);
        }
        break;
    case 0x0070:   // слово(2) + значение(4)
        for (uint16_t i = 0; i + 6 <= n; i += 6) {
            const uint16_t word = get16(v + i);
            const uint32_t val = get32(v + i + 2);
            if (word == 0x05 && val >= 1 && val <= SENSOR_FAR_MAX) {
                farGate = val;
            } else if (word == 0x06 && val >= OFF_DELAY_MIN_S && val <= OFF_DELAY_MAX_S) {
                offDelayS = val;
            } else if (word == 0x0B && (val == 5 || val == 10)) {
                responseSpeed = val;
            } else if (word == 0x02 && val >= 5 && val <= 80 && val % 5 == 0) {
                statusRateX10 = val;
            } else {
                status = 1;
                simSay("датчик: 0x0070 неверно - слово 0x%02X = %u", word, val);
            }
        }
        break;
    case 0x0071:
        for (uint16_t i = 0; i + 2 <= n; i += 2) {
            const uint16_t word = get16(v + i);
            put32(d, word == 0x05 ? farGate : word == 0x06 ? offDelayS : word == 0x02 ? statusRateX10 : word == 0x0C ? 5 :
                     word == 0x0B ? responseSpeed : 0);
        }
        break;
    case 0x0072:
    case 0x0076:
        for (uint16_t i = 0; i + 6 <= n; i += 6) {
            const uint16_t g = get16(v + i);
            if (g < SENSOR_GATES) {
                (cmd == 0x0072 ? trig : hold)[g] = get32(v + i + 2);
            } else {
                status = 1;
            }
        }
        break;
    case 0x0073:
    case 0x0077:
        for (uint16_t i = 0; i + 2 <= n; i += 2) {
            const uint16_t g = get16(v + i);
            put32(d, g < SENSOR_GATES ? (cmd == 0x0073 ? trig : hold)[g] : 0);
        }
        break;
    case 0x007A:
        if (n >= 6) {
            standard = get32(v + 2) == 1;
        }
        break;
    case 0x0009:
        if (n >= 6) {   // коэффициенты, время скана, с; 100 % - через 123 с при 120 с (перехват)
            autoActive = true;
            autoDone = false;
            autoStartUs = g_simUs + SIM_ACK_US;
            autoEndUs = autoStartUs + (uint64_t)get16(v + 4) * 1000000 * 123 / 120;
            for (uint8_t g = 0; g < SENSOR_GATES; g++) {
                autoMaxDb[g] = 0;
            }
        }
        break;
    default:
        status = 1;
        break;
    }
    return d;
}
