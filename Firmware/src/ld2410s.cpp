// Драйвер HLK-LD2410S. Формат - малый порядок байт (п. 2), ссылки на пункты протокола в комментариях.
#include "ld2410s.h"

constexpr uint8_t  CMD_HEAD[4]   = {0xFD, 0xFC, 0xFB, 0xFA};   // заголовок команды и ACK (п. 2.2)
constexpr uint8_t  CMD_TAIL[4]   = {0x04, 0x03, 0x02, 0x01};   // конец команды и ACK
constexpr uint8_t  REP_HEAD[4]   = {0xF4, 0xF3, 0xF2, 0xF1};   // заголовок стандартного отчёта (табл. 2-1)
constexpr uint8_t  REP_TAIL[4]   = {0xF8, 0xF7, 0xF6, 0xF5};   // конец стандартного отчёта
constexpr uint8_t  MINREP_HEAD   = 0x6E;   // начало минимального отчёта (табл. 2-1)
constexpr uint8_t  MINREP_TAIL   = 0x62;   // конец минимального отчёта
constexpr uint8_t  MINREP_LEN    = 5;      // длина минимального отчёта: 6E, состояние, расстояние(2), 62
constexpr uint8_t  HDR_LEN       = 6;      // заголовок(4) + длина данных(2)
constexpr uint8_t  TAIL_LEN      = 4;      // конец кадра
constexpr size_t   FRAME_MAX     = 96;     // самый длинный ожидаемый кадр (ACK 0x0073: 6 + 68 + 4), байт
constexpr uint16_t ACK_FLAG      = 0x0100; // ACK = слово команды | 0x0100 (по примерам п. 2.2)
constexpr uint8_t  REP_STANDARD  = 0x01;   // тип стандартного отчёта
constexpr uint8_t  REP_PROGRESS  = 0x03;   // тип отчёта хода автокалибровки
constexpr uint8_t  STD_ENERGY_OFFSET = 6;  // смещение энергий в данных стандартного отчёта: тип, состояние,
                                           // расстояние(2), резерв(2)
constexpr uint16_t PAR_FAR_GATE  = 0x05;   // слово «дальние ворота» (табл. 2-2)
constexpr uint16_t PAR_OFF_DELAY = 0x06;   // слово «время без человека», с (табл. 2-2)

static Stream* s_uart = nullptr;           // UART датчика
static uint8_t s_buf[FRAME_MAX];           // собираемый кадр
static size_t  s_len = 0;                  // принято байт кадра
static size_t  s_frameNeed = 0;            // полная длина кадра (0 - ещё неизвестна)
static size_t  s_frameLen = 0;             // длина последнего собранного кадра в s_buf, байт
static const char* s_lastError = "";       // причина неудачи последней команды (пусто - успех)
static uint16_t s_rxBytes = 0;             // принято байт с начала ожидания ACK последней команды
static char    s_errText[64];              // текст «нет ответа» с числом принятых байт

void ldBegin(Stream& uart) {
    s_uart = &uart;
    s_len = 0;
    s_frameNeed = 0;
}

static uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
                                                ((uint32_t)p[3] << 24); }
static void wr16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t* p, uint32_t v) { wr16(p, (uint16_t)v); wr16(p + 2, (uint16_t)(v >> 16)); }

// Добавляет байт к кадру. true - кадр собран и проверен: он в s_buf, длина - s_frameLen
// (действителен до следующего вызова).
static bool feedByte(uint8_t b) {
    if (s_len == 0 && b != CMD_HEAD[0] && b != REP_HEAD[0] && b != MINREP_HEAD) {
        return false;
    }
    s_buf[s_len++] = b;
    if (s_buf[0] == MINREP_HEAD) {
        if (s_len < MINREP_LEN) {
            return false;
        }
        s_len = 0;
        s_frameLen = MINREP_LEN;
        return b == MINREP_TAIL;
    }
    const bool isCmd = s_buf[0] == CMD_HEAD[0];
    if (s_len <= sizeof(CMD_HEAD) && b != (isCmd ? CMD_HEAD : REP_HEAD)[s_len - 1]) {   // не заголовок
        s_len = 0;
        return false;
    }
    if (s_len == HDR_LEN) {
        s_frameNeed = HDR_LEN + rd16(s_buf + 4) + TAIL_LEN;
        if (s_frameNeed > FRAME_MAX) {
            s_len = 0;
        }
        return false;
    }
    if (s_len < HDR_LEN || s_len < s_frameNeed) {
        return false;
    }
    s_frameLen = s_frameNeed;
    s_len = 0;
    return memcmp(s_buf + s_frameLen - TAIL_LEN, isCmd ? CMD_TAIL : REP_TAIL, TAIL_LEN) == 0;
}

// Следующий собранный кадр из принятых байт: true, кадр в s_buf, длина в len.
static bool nextFrame(size_t& len) {
    while (s_uart && s_uart->available()) {
        s_rxBytes++;
        if (feedByte((uint8_t)s_uart->read())) {
            len = s_frameLen;
            return true;
        }
    }
    return false;
}

static bool parseReport(size_t len, LdReport& r) {
    if (s_buf[0] == MINREP_HEAD) {
        r.type = 0;
        r.present = s_buf[1] >= 2;
        r.distanceCm = rd16(s_buf + 2);
        return true;
    }
    if (s_buf[0] != REP_HEAD[0] || len < HDR_LEN + 1 + TAIL_LEN) {
        return false;
    }
    const uint8_t* d = s_buf + HDR_LEN;
    const uint16_t dlen = rd16(s_buf + 4);
    r.type = d[0];
    if (r.type == REP_PROGRESS && dlen >= 3) {
        r.progress = rd16(d + 1);
        return true;
    }
    if (r.type == REP_STANDARD && dlen >= STD_ENERGY_OFFSET + SENSOR_GATES * 4) {
        r.present = d[1] >= 2;
        r.distanceCm = rd16(d + 2);
        for (uint8_t g = 0; g < SENSOR_GATES; g++) {
            r.energy[g] = rd32(d + STD_ENERGY_OFFSET + g * 4);
        }
        return true;
    }
    return false;
}

bool ldPoll(LdReport& r) {
    size_t len;
    while (nextFrame(len)) {
        if (parseReport(len, r)) {
            return true;
        }
    }
    return false;
}

// Отправляет команду и ждёт ACK. В ack - данные ACK после слова команды (статус и значения).
static bool command(uint16_t cmd, const uint8_t* val, uint16_t valLen, const uint8_t** ack = nullptr,
                    uint16_t* ackLen = nullptr) {
    if (!s_uart) {
        return false;
    }
    uint8_t lenCmd[4];
    wr16(lenCmd, (uint16_t)(2 + valLen));
    wr16(lenCmd + 2, cmd);
    s_uart->write(CMD_HEAD, sizeof(CMD_HEAD));
    s_uart->write(lenCmd, sizeof(lenCmd));
    if (valLen) {
        s_uart->write(val, valLen);
    }
    s_uart->write(CMD_TAIL, sizeof(CMD_TAIL));
    s_uart->flush();

    const uint32_t start = millis();
    s_rxBytes = 0;
    size_t len;
    while (millis() - start < CMD_TIMEOUT_MS) {
        if (!nextFrame(len) || s_buf[0] != CMD_HEAD[0] || rd16(s_buf + HDR_LEN) != (cmd | ACK_FLAG)) {
            continue;
        }
        const uint16_t dlen = rd16(s_buf + 4);
        if (ack) {
            *ack = s_buf + HDR_LEN + 2;
            *ackLen = (uint16_t)(dlen - 2);
        }
        const bool ok = dlen < 4 || rd16(s_buf + HDR_LEN + 2) == 0;   // статус 0 - успешно (если он есть в ACK)
        s_lastError = ok ? "" : "отказ - статус не 0";
        return ok;
    }
    snprintf(s_errText, sizeof(s_errText), "нет ответа (принято байт %u)", s_rxBytes);   // 0 - датчик молчит
    s_lastError = s_errText;
    return false;
}

const char* ldLastError() {
    return s_lastError;
}

bool ldEnableConfig(uint16_t* proto, uint16_t* bufSize) {
    uint8_t v[2];
    wr16(v, 0x0001);   // значение команды (п. 2.2.3)
    const uint8_t* ack;
    uint16_t ackLen = 0;
    const bool ok = command(0x00FF, v, sizeof(v), &ack, &ackLen);
    if (ok && proto && ackLen >= 6) {   // ACK: статус(2), версия протокола(2), размер буфера(2)
        *proto = rd16(ack + 2);
        *bufSize = rd16(ack + 4);
    }
    return ok;
}

uint8_t ldQuery(uint16_t cmd, uint8_t* out, uint8_t max) {
    const uint8_t* ack;
    uint16_t ackLen = 0;
    command(cmd, nullptr, 0, &ack, &ackLen);   // статус не проверяем - его разбирает вызывающий
    const uint8_t n = ackLen < max ? (uint8_t)ackLen : max;
    if (n) {
        memcpy(out, ack, n);
    }
    return n;
}

bool ldReadSerial(char sn[LD_SN_LEN + 1]) {
    const uint8_t* ack;
    uint16_t ackLen = 0;
    if (!command(0x0011, nullptr, 0, &ack, &ackLen) || ackLen < 4) {   // статус(2), длина(2), номер (п. 2.2.6)
        return false;
    }
    uint16_t n = rd16(ack + 2);
    n = n > LD_SN_LEN ? LD_SN_LEN : n;
    n = n > ackLen - 4 ? ackLen - 4 : n;
    memcpy(sn, ack + 4, n);
    sn[n] = '\0';
    return true;
}

bool ldReadParams(const uint16_t* words, uint8_t count, uint32_t* out) {
    uint8_t v[LD_PARAMS_MAX * 2];
    for (uint8_t i = 0; i < count; i++) {
        wr16(v + i * 2, words[i]);
    }
    const uint8_t* ack;
    uint16_t ackLen = 0;
    if (!command(0x0071, v, count * 2, &ack, &ackLen) || ackLen < 2 + count * 4) {   // статус(2), по 4 байта
        return false;                                                               // (п. 2.2.8)
    }
    for (uint8_t i = 0; i < count; i++) {
        out[i] = rd32(ack + 2 + i * 4);
    }
    return true;
}

bool ldEndConfig() {
    return command(0x00FE, nullptr, 0);
}

bool ldSetStandardMode(bool standard) {
    uint8_t v[6];
    wr16(v, 0x0000);
    wr32(v + 2, standard ? 1 : 0);   // 000000010000 - стандартный, 000000000000 - минимальный (п. 2.2.1)
    return command(0x007A, v, sizeof(v));
}

bool ldWriteParams(uint8_t farGate, uint32_t offDelayS) {
    uint8_t v[12];
    uint8_t len = 0;
    if (farGate) {   // 0 - дальние ворота не писать (заводские); пар «слово + значение» - любое число (инстр. п. 2.2.7)
        wr16(v, PAR_FAR_GATE);
        wr32(v + 2, farGate);
        len = 6;
    }
    wr16(v + len, PAR_OFF_DELAY);
    wr32(v + len + 2, offDelayS);
    return command(0x0070, v, len + 6);
}

bool ldStartAutoThreshold(uint16_t trig, uint16_t hold, uint16_t scanS) {
    uint8_t v[6];
    wr16(v, trig);
    wr16(v + 2, hold);
    wr16(v + 4, scanS);
    return command(0x0009, v, sizeof(v));
}

bool ldReadThresholds(uint16_t cmd, uint32_t out[SENSOR_GATES]) {
    uint8_t v[SENSOR_GATES * 2];
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        wr16(v + g * 2, g);
    }
    for (uint8_t i = 0; i < CMD_RETRIES; i++) {   // повтор: сразу после автокалибровки датчик может не ответить
        const uint8_t* ack;
        uint16_t ackLen = 0;
        if (!command(cmd, v, sizeof(v), &ack, &ackLen)) {
            continue;
        }
        if (ackLen < 2 + SENSOR_GATES * 4) {
            s_lastError = "короткий ответ";
            continue;
        }
        for (uint8_t g = 0; g < SENSOR_GATES; g++) {
            out[g] = rd32(ack + 2 + g * 4);   // после статуса(2) - значения по 4 байта (п. 2.2.11)
        }
        return true;
    }
    return false;
}

bool ldWriteThresholds(uint16_t cmd, const uint32_t in[SENSOR_GATES]) {
    uint8_t v[SENSOR_GATES * 6];
    for (uint8_t g = 0; g < SENSOR_GATES; g++) {
        wr16(v + g * 6, g);
        wr32(v + g * 6 + 2, in[g]);
    }
    return command(cmd, v, sizeof(v));
}
