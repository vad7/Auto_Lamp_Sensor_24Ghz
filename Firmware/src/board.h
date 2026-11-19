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
enum class Led : uint8_t { Off, Red, Green, Blue, CalLeave, NoSensor, SupplyFault, StuckFlash };   // CalLeave -
    // пауза перед калибровкой (красный / зелёный), NoSensor - ошибка калибровки / нет связи с датчиком, SupplyFault -
    // обрыв D1, Blue - вспышки, StuckFlash - залипание датчика (белая вспышка LED_STUCK_FLASH_MS раз в
    // LED_STUCK_PERIOD_MS)

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
    uint16_t lightDarkLevel;        // калибровка: свет «темно» (шаг 1), отсчёты АЦП; 0 и lightLitLevel 0 - не мерили
    uint16_t lightLitLevel;         // калибровка: свет «светло» (шаг 2), отсчёты АЦП
    uint16_t stuckMinutes;          // залипание: лампа горит без перерыва, мин; 0 - не проверять
    uint8_t  stuckLightPct;         // залипание: свет не меняется в пределах % от |lightLitLevel - lightDarkLevel|
    uint8_t  stuckDeltaDb;          // залипание: лампа вкл, если энергия ворот выше фона на, дБ
    uint32_t nightPwmHz;            // ночник: частота ШИМ, Гц: NIGHT_PWM_HZ_MIN..NIGHT_PWM_HZ_MAX (консоль pwm)
    uint8_t  standbyLevel;          // ночник: дежурная подсветка - 0 выкл, 1..STANDBY_LEVEL_COUNT - уровень
    uint8_t  standbyPct[STANDBY_LEVEL_COUNT];   // ночник: яркость уровней дежурки, %
    uint16_t standbyOnThreshold;    // дежурка не горит: включить, если темнее, отсчёты АЦП
    uint16_t standbyOffThreshold[STANDBY_LEVEL_COUNT];   // дежурка горит на уровне (её свет на фоторезисторе):
                                                         // погасить, если светлее, отсчёты АЦП
    uint16_t lightThresholdStandby[STANDBY_LEVEL_COUNT]; // порог основного света, пока горит дежурка на уровне
                                                         // (фоторезистор видит её), отсчёты АЦП
    uint16_t lightStandbyLevel[STANDBY_LEVEL_COUNT];     // калибровка: «темно» с дежуркой на уровне, отсчёты АЦП;
                                                         // 0 - не мерили
};

// Пороги датчика заданы калибровкой (или консолью «G: T / H»): порог 0 не бывает (диапазон 10..95 дБ, программа HLK).
inline bool sensorCalibrated(const Calib& c) {
    return c.trig[0] != 0;
}

constexpr uint32_t CALIB_MAGIC   = 0x4C414D50;  // «LAMP»
constexpr uint16_t CALIB_VERSION_NO_PWM = 6;    // прежняя версия: структура до nightPwmHz - переводится при загрузке
constexpr uint16_t CALIB_VERSION_NO_STANDBY = 7;   // прежняя версия: структура до standbyLevel - переводится при загрузке
constexpr uint16_t CALIB_VERSION_ONE_STANDBY = 8;  // прежняя версия: пороги дежурки одни на все уровни - при загрузке
                                                   // уровень и проценты дежурки остаются, пороги - заново от света
constexpr uint16_t CALIB_VERSION = 9;       // менять при изменении Calib

Stream*  boardConsoleOpen();                 // при включении: перемычка USB замкнута - индикатор (синий), поток
                                             // консоли; перемычки нет - nullptr
bool     boardConsoleConnected();            // монитор порта USB ещё открыт
#ifdef NIGHT_LIGHT
void     boardLampFreq(uint32_t hz);         // ночник: частота ШИМ, Гц, ночник гаснет; после boardInit - снова
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
