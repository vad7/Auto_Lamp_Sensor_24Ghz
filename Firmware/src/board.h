// Слой платы: всё, что отличается у RP2040-Zero / ESP32-C3 / ESP32-C6. Реализация - board_<плата>.cpp.
#pragma once
#include <Arduino.h>
#include "config.h"

#ifdef DEBUG_LOG
#define LOG(...) Serial.printf(__VA_ARGS__)
#else
#define LOG(...) ((void)0)
#endif

// Состояния индикатора: на RP2040-Zero - цвет WS2812B, на одноцветных платах ESP32 - свечение / мигание
enum class Led : uint8_t { Off, Red, Green, Blue, CalLeave, NoSensor, SupplyFault };   // CalLeave - пауза перед
    // калибровкой (красный / зелёный), NoSensor - ошибка калибровки / нет связи с датчиком, SupplyFault - обрыв D1,
    // Blue - вспышки

// Данные калибровки, хранятся во флеш-памяти МК. Не упакована: на Cortex-M0+ невыровненный доступ к uint32_t -
// аварийная остановка; поля расставлены без выравнивающих пропусков.
struct Calib {
    uint32_t magic;                 // признак записи CALIB_MAGIC
    uint32_t trig[16];              // пороги срабатывания ворот 0..15 (ответ команды 0x0073)
    uint32_t hold[16];              // пороги удержания ворот 0..15 (ответ команды 0x0077)
    uint16_t version;               // версия структуры CALIB_VERSION
    uint16_t lightThreshold;        // порог освещённости между «темно» и «лампа горит», отсчёты АЦП
    uint8_t  farGate;               // параметр «дальние ворота» (0x0070, слово 0x05), 1..16; 0 - датчик не калиброван
    LdrInputMode ldrInputMode;      // фоторезистор: нет / прямая / обратная кривая (1 байт)
    uint8_t  nightLevel;            // ночник: выбранный уровень яркости, индекс в nightPct
    uint8_t  offDelay;              // задержка «нет человека» (0x0070, слово 0x06), с: OFF_DELAY_MIN_S..OFF_DELAY_MAX_S
    uint8_t  nightPct[NIGHT_LEVEL_COUNT];   // ночник: яркость уровней, % (заполнение ШИМ); уровень 0 - минимум при перегреве
    int8_t   tempMax;               // ночник: порог перегрева, °C (с поправкой NIGHT_TEMP_OFFSET_C)
    uint16_t minOnS;                // свет горит не меньше, с, от последнего «человек есть» на OT2 (против дребезга)
};

// Пороги датчика заданы калибровкой (или консолью trig): порог 0 не бывает (диапазон 10..95 дБ, программа HLK).
inline bool sensorCalibrated(const Calib& c) {
    return c.trig[0] != 0;
}

constexpr uint32_t CALIB_MAGIC   = 0x4C414D50;  // «LAMP»
constexpr uint16_t CALIB_VERSION = 5;           // менять при изменении Calib

Stream*  boardConsoleOpen();                 // при включении: перемычка USB замкнута - индикатор (синий), поток
                                             // консоли; перемычки нет - nullptr
bool     boardConsoleConnected();            // монитор порта USB ещё открыт
#ifdef NIGHT_LIGHT
void     boardConsoleLamp(uint8_t percent);  // консоль: ночник с яркостью, % (0 - выкл); до boardInit
#endif
void     boardInit();                        // такты (USB выкл), пины, индикатор
void     boardSleep(uint32_t ms, bool deep);  // ожидание; deep - сон с остановкой тактов (UART не работает)
bool     boardButton();                      // кнопка калибровки нажата
bool     boardPresence();                    // выход OT2 датчика
void     boardLamp(uint8_t percent);         // лампа: 0 - выкл; ключ 230 В - вкл при > 0, ночник - яркость, %
uint16_t boardLight();                       // освещённость, отсчёты АЦП (среднее LIGHT_SAMPLES)
uint16_t boardSupplyMv();                    // напряжение на стабилитронах блока питания, мВ
int16_t  boardTempC();                       // температура кристалла МК (встроенный датчик), °C
void     boardLed(Led led);                  // режим индикатора
void     boardLedTick(uint32_t now);         // мигание индикатора, вызывать каждый проход
Stream&  boardSensorBegin();                 // включить UART датчика
void     boardSensorEnd();                   // выключить UART датчика
bool     boardLoad(Calib& c);                // прочитать калибровку; false - нет или старая версия
void     boardSave(const Calib& c);          // записать калибровку
