// Драйвер датчика HLK-LD2410S по UART (Docs/HLK-LD2410S_Serial_communication_protocol_V1.00.pdf).
#pragma once
#include <Arduino.h>
#include "config.h"

// Отчёт датчика (табл. 2-1)
struct LdReport {
    uint8_t  type;                    // 0 - минимальный, 1 - стандартный, 3 - ход автокалибровки
    bool     present;                 // состояние 2/3 - человек есть
    uint16_t distanceCm;              // расстояние до цели, см
    uint32_t energy[SENSOR_GATES];    // энергия по воротам (только стандартный отчёт)
    uint16_t progress;                // ход автокалибровки, % (перехват; в табл. 2-1 - % * 100), только тип 3
};

// Команды (п. 2.2). Каждая ждёт ACK не дольше CMD_TIMEOUT_MS; true - датчик ответил «успешно».
// Пришедшие за это время отчёты отбрасываются.
constexpr uint8_t LD_SN_LEN     = 8;   // серийный номер, символов (п. 2.2.5)
constexpr uint8_t LD_PARAMS_MAX = 8;   // ldReadParams: параметров за один запрос

void ldBegin(Stream& uart);
const char* ldLastError();   // причина неудачи последней команды: «нет ответа (принято байт N)», «отказ - статус не 0»,
                             // «короткий ответ»
bool ldEnableConfig(uint16_t* proto = nullptr, uint16_t* bufSize = nullptr);   // 0x00FF; версия протокола и буфер
uint8_t ldQuery(uint16_t cmd, uint8_t* out, uint8_t max);           // команда без значения: байты ACK после слова
                                                                    // команды (не больше max), 0 - нет ответа
// Команды без значения, перехват обмена программы HLK-LD2410S_TOOL v1.3.0.1 (work/uart_sniff_*.log): ответы -
// статус(2) + данные
constexpr uint16_t LD_CMD_VERSION    = 0x0000;  // версия: 2 слова (у программы «_32768.0») + 3 слова (v1.1.1); п. 2.2.2
constexpr uint16_t LD_CMD_FW_INFO_60 = 0x0060;  // Obtain firmware info: 4 байта (у датчика 2), в протоколе нет
constexpr uint16_t LD_CMD_FW_INFO_61 = 0x0061;  // Obtain firmware info: 4 байта (у датчика 1), в протоколе нет
constexpr uint16_t LD_CMD_DEVICE_ID  = 0x0065;  // Device ID: 4 байта (у датчика «L4PA» = 0x4150344C), в протоколе нет
bool ldReadSerial(char sn[LD_SN_LEN + 1]);                          // 0x0011: серийный номер (строка)
bool ldReadParams(const uint16_t* words, uint8_t count, uint32_t* out);   // 0x0071: значения слов (табл. 2-2)
bool ldEndConfig();                                                 // 0x00FE
bool ldSetStandardMode(bool standard);                              // 0x007A
bool ldWriteParams(uint8_t farGate, uint32_t offDelayS);           // 0x0070: слова 0x05 (если не 0), 0x06
bool ldStartAutoThreshold(uint16_t trig, uint16_t hold, uint16_t scanS);  // 0x0009
bool ldReadThresholds(uint16_t cmd, uint32_t out[SENSOR_GATES]);    // 0x0073 / 0x0077
bool ldWriteThresholds(uint16_t cmd, const uint32_t in[SENSOR_GATES]);    // 0x0072 / 0x0076

// Разбор принятых байт без ожидания; true - получен очередной отчёт.
bool ldPoll(LdReport& r);

constexpr uint16_t LD_CMD_WRITE_TRIG = 0x0072;   // запись порогов срабатывания (п. 2.2.10)
constexpr uint16_t LD_CMD_READ_TRIG  = 0x0073;   // чтение порогов срабатывания (п. 2.2.11)
constexpr uint16_t LD_CMD_WRITE_HOLD = 0x0076;   // запись порогов удержания (п. 2.2.12)
constexpr uint16_t LD_CMD_READ_HOLD  = 0x0077;   // чтение порогов удержания (п. 2.2.13)
