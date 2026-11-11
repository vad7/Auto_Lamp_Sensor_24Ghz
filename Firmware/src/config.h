// Настройки прошивки: пины плат, тайминги, константы. Источники - Readme.md (распиновка), Docs/.
#pragma once
#include <stdint.h>

constexpr char FW_VERSION[] = "1.00";   // версия прошивки

// Экономия питания: 1 - после консоли USB выключается, такты 12 МГц от кварца, ядро 1,05 В, сон с остановкой тактов
// (нужно лампе 230 В: конденсаторный блок питания, Readme «Блок питания»); 0 - такты и USB ядра по умолчанию, сон
// без остановки тактов (проще, ток больше). Отладка (DEBUG_LOG) - всегда как 0.
#define LOW_POWER 0

// --- Пины (Readme.md, «Распиновка») ---
#if defined(ARDUINO_ARCH_RP2040)
// Waveshare RP2040-Zero (Docs/RP2040-Zero.pdf, Docs/RP2040-Zero_Schematic.pdf)
constexpr uint8_t PIN_SENSOR_TX  = 0;    // GP0, UART0 TX -> RX датчика LD2410S
constexpr uint8_t PIN_SENSOR_RX  = 1;    // GP1, UART0 RX <- OT1 (TX) датчика
constexpr uint8_t PIN_PRESENCE   = 2;    // GP2 <- OT2 датчика: 1 - есть человек
constexpr uint8_t PIN_LAMP       = 4;    // GP4: лампа 230 В - R2 180 Ом -> светодиод MOC3043M (1 - горит);
                                         //      ночник - ШИМ -> R2 100 Ом -> затвор P75N02LDG
constexpr uint8_t PIN_LIGHT      = 26;   // GP26 (ADC0) <- делитель фоторезистора
constexpr uint8_t LIGHT_ADC_CH   = 0;    // канал АЦП вывода GP26
constexpr uint8_t PIN_SUPPLY     = 28;   // GP28 (ADC2) <- делитель R5 / R6 с узла стабилитронов блока питания
constexpr uint8_t SUPPLY_ADC_CH  = 2;    // канал АЦП вывода GP28
constexpr uint8_t TEMP_ADC_CH    = 4;    // канал АЦП встроенного термодатчика (п. 4.9.5 даташита RP2040)
constexpr uint8_t PIN_LED        = 16;   // GP16 -> DIN WS2812B на плате (L1 на схеме)
constexpr uint8_t PIN_USB_JUMPER = 7;    // GP7 (подтяжка к 3V3) - перемычка - GP8: замкнута при включении - консоль USB
constexpr uint8_t PIN_USB_JUMPER_LOW = 8;   // GP8: 0 на время проверки перемычки (угол платы рядом с GP7)
// Кнопка калибровки - BOOT (не GPIO, читается объектом BOOTSEL ядра)
#else
#error "Плата не поддержана: добавить пины ESP32-C3 / ESP32-C6 по их схемам"
#endif

// --- Датчик LD2410S (Docs/HLK-LD2410S_Serial_communication_protocol_V1.00.pdf) ---
constexpr uint32_t SENSOR_BAUD       = 115200;  // скорость UART датчика по умолчанию, бод (инстр. п. 2)
constexpr uint8_t  SENSOR_GATES      = 16;      // ворот дальности 0..15 (инстр. п. 2.2.10)
constexpr uint8_t  SENSOR_FAR_MAX    = 16;      // максимум параметра «дальние ворота» (инстр. табл. 2-2)
constexpr uint16_t AUTO_TRIG_FACTOR  = 2;       // коэффициент срабатывания автокалибровки (инстр., пример п. 2.2.9)
constexpr uint16_t AUTO_HOLD_FACTOR  = 1;       // коэффициент удержания автокалибровки (инстр., пример п. 2.2.9)
constexpr uint8_t  OFF_DELAY_S       = 30;      // «нет человека» -> OT2 = 0 через, с - по умолчанию (меняется кнопкой)
constexpr uint8_t  OFF_DELAY_MIN_S   = 10;      // минимум задержки, с (инстр. табл. 2-2)
constexpr uint8_t  OFF_DELAY_MAX_S   = 120;     // максимум задержки, с (инстр. табл. 2-2)
constexpr uint8_t  OFF_DELAY_STEP_S  = 10;      // шаг настройки задержки кнопкой, с (одна вспышка индикатора)
constexpr uint16_t MIN_ON_DEFAULT_S  = 30;      // свет горит не меньше, с, от последнего импульса OT2 - по умолчанию
                                                // (настройка по USB minon, 0..65535; 0 - выкл сразу по OT2)
constexpr uint32_t CMD_TIMEOUT_MS    = 300;     // ожидание ответа (ACK) на команду, мс
constexpr uint8_t  CMD_RETRIES       = 3;       // попыток чтения порогов (3 x 300 мс - в окне 3 с после входа в режим
                                                // команд: Operation Guide п. 4.1.2)
constexpr uint8_t  SENSOR_ATTEMPTS   = 3;       // попыток записи настроек в датчик, затем ошибка связи (задал пользователь)
constexpr uint32_t SENSOR_RETRY_MS   = 10000;   // пауза между попытками, мс (время готовности датчика после включения неизвестно)

// --- Калибровка (Readme.md, «Калибровка») ---
constexpr uint32_t CAL_LEAVE_S       = 10;      // перед шагом 1: время выйти из комнаты, с
constexpr uint32_t CAL_PRESENCE_S    = 30;      // длительность шага 2 калибровки (присутствие), с (задал пользователь)
constexpr uint32_t CAL_SCAN_S        = 120;     // шаг 1: сканирование автокалибровки датчика, с (минимум 120 -
                                                // программа HLK-LD2410S_TOOL: «MinValue Must >= 120»; пример п. 2.2.9)
constexpr uint32_t CAL_SCAN_MARGIN_S = 5;       // запас после сканирования автокалибровки, с (шаг 1 - 125 с, задал
                                                // пользователь): по перехвату 100 % через 123 с, затем датчик ~3 с не
                                                // отвечает и сбивается на 1-2 команды - команды шага 2 идут позже
constexpr uint32_t CAL_READ_PAUSE_MS = 5000;    // после шага 1 до первой попытки прочитать пороги, мс (задал пользователь)
constexpr uint8_t  CAL_READ_TRIES    = 10;      // попыток прочитать пороги и включить стандартный отчёт: на железе датчик
                                                // ответил через ~8-9 с после 100 % (4-я попытка раз в 1 с)
constexpr uint32_t CAL_READ_RETRY_MS = 1000;    // между попытками, мс
constexpr uint8_t  CAL_PRESENT_PCT   = 80;      // шаг 2: датчик должен видеть человека не меньше, % кадров
constexpr uint16_t CAL_MIN_FRAMES    = 10;      // шаг 2: минимум кадров от датчика (иначе датчика нет)
constexpr uint8_t  CAL_GATE_HIT_PCT  = 10;      // ворота «в комнате», если энергия выше порога в >= % кадров
constexpr uint32_t CAL_LIGHT_STEP_MS = 500;     // период замеров фоторезистора на шагах калибровки, мс
constexpr uint32_t CAL_LIGHT_WINDOW_MS = 10000; // конец шага: индикатор погашен (его свет врёт фоторезистору), замеры, мс
constexpr uint32_t CAL_LIGHT_SETTLE_MS = 1000;  // от гашения индикатора до первого замера, мс: инерция фоторезистора - оценка
constexpr uint16_t LIGHT_MIN_DIFF    = 100;     // минимальная разница «темно / лампа горит», отсчёты АЦП
// Фоторезистор (Calib.ldrInputMode, консоль ldr 0|1|2). Значения хранятся во флеш - не переставлять.
enum LdrInputMode : uint8_t {
    LDR_NONE      = 0,   // нет фоторезистора: свет не проверяется, всегда «темно»
    LDR_DARK_LOW  = 1,   // прямая кривая: больше света - больше отсчёт АЦП (темно - малый отсчёт)
    LDR_DARK_HIGH = 2,   // обратная кривая: больше света - меньше отсчёт АЦП (темно - большой отсчёт)
};
// Фоторезистор установлен (по умолчанию - первое включение, reset): 1 - есть, 0 - нет; потом - консоль ldr.
#define LDR_EXISTS 1
constexpr uint32_t CAL_OK_FLASH_MS   = 400;     // успех калибровки: каждый цвет (красный, синий, зелёный), мс
constexpr uint32_t CAL_FAIL_FLASH_MS = 200;     // ошибка калибровки: вспышка красным и пауза, мс
constexpr uint8_t  CAL_FAIL_FLASHES  = 5;       // ошибка калибровки: число вспышек
constexpr float    ENERGY_DB_SCALE   = 10.0f;   // энергия ворот (линейная) -> дБ порогов: 10 * log10(E) (перехват)
constexpr uint16_t CAL_PROGRESS_STEP = 10;      // ход автокалибровки датчика в USB - каждые 10 %
constexpr uint8_t  CAL_MSG_LEN       = 200;     // строка сообщения о ходе калибровки в USB, байт

// --- Без калибровки (первое включение): датчик с заводскими порогами, свет - по умолчанию. Подобрать на железе ---
constexpr uint16_t LIGHT_DEFAULT_THRESHOLD = 2048;  // порог света, отсчёты АЦП: середина шкалы (R1 равен
                                                    // фоторезистору на пороге, Readme «Детали»)
constexpr LdrInputMode LDR_DIVIDER_MODE = LDR_DARK_LOW;   // режим при LDR_EXISTS: делитель - LDR к 3V3, R1 к GND

// --- Кнопка BOOT ---
constexpr uint32_t SETUP_PRESS_MS    = 5000;    // удержание: вспышка, отпустили - настройка задержки выключения, мс
constexpr uint32_t CAL_PRESS_MS      = 10000;   // удержание: запуск калибровки (сразу, не дожидаясь отпускания), мс
constexpr uint32_t DELAY_DEC_PRESS_MS = 1000;   // настройка задержки: нажатие короче - плюс шаг, дольше - минус, мс
constexpr uint32_t DELAY_EXIT_MS     = 30000;   // настройка задержки: без нажатий - сохранить и выйти, мс
constexpr uint32_t FLASH_ON_MS       = 200;     // вспышка индикатора (синий), мс
constexpr uint32_t FLASH_OFF_MS      = 400;     // промежуток между вспышками в счёте, мс
constexpr uint32_t FLASH_PAUSE_MS    = 3000;    // пауза между повторами счёта вспышек, мс

// --- Настройка по USB (консоль в мониторе порта; при включении замкнута перемычка PIN_USB_JUMPER) ---
constexpr uint32_t JUMPER_SETTLE_US  = 10;      // установление подтяжки перед чтением перемычки, мкс
constexpr uint32_t USB_POLL_MS       = 10;      // период опроса USB в консоли, мс
constexpr uint32_t CONSOLE_IDLE_MS   = 120000;   // выход из консоли без команд (от включения или последней команды), мс
constexpr uint8_t  CONSOLE_LINE      = 48;      // длина строки команды консоли, символов
constexpr uint32_t CONSOLE_FLUSH_MS  = 100;     // консоль: ожидание остатка строки (\r\n) перед вопросом, мс
constexpr uint32_t MONITOR_PRINT_MS  = 1000;    // монитор консоли (mon): период строки состояния, мс
constexpr uint32_t DIAG_REPORTS_MS   = 2000;    // диагностика датчика (sensor): приём отчётов, мс
constexpr uint32_t DIAG_RAW_MS       = 1000;    // диагностика датчика: приём сырых байт до разбора кадров, мс
constexpr uint8_t  DIAG_RAW_SHOW     = 16;      // диагностика датчика: показать первых байт (HEX)
constexpr uint32_t BOOT_FLASH_MS     = 300;     // вспышка индикатора (зелёный) при включении, мс

// --- Работа ---
constexpr uint32_t TICK_MS           = 20;      // период опроса кнопки и OT2 (между опросами - сон), мс
constexpr uint32_t LIGHT_RECHECK_MS  = 5000;    // повторный замер света, пока человек есть, а лампа не горит, мс
constexpr uint8_t  LIGHT_SAMPLES     = 8;       // замеров АЦП на одно значение освещённости
constexpr uint8_t  LED_LEVEL         = 40;      // яркость индикатора, 0..255
constexpr uint32_t LED_BLINK_MS      = 400;     // мигание индикатора (ошибка и пауза калибровки): горит и не горит по, мс
// --- Контроль стабилитронов блока питания (Readme.md, «Блок питания») ---
constexpr uint32_t ADC_VREF_MV       = 3300;    // опорное АЦП - 3V3 платы (ADC_VREF через R6 200 Ом), мВ
constexpr uint32_t ADC_FULL          = 4095;    // отсчёт АЦП при входе = опорному (12 бит)
constexpr uint32_t SUPPLY_R_TOP      = 100;     // R5, верхнее плечо делителя, кОм
constexpr uint32_t SUPPLY_R_BOTTOM   = 150;     // R6, нижнее плечо делителя, кОм
constexpr uint16_t SUPPLY_FAULT_MV   = 4550;    // выше - D1 оборван, держит D2 (середина 4,38 / 4,74 В, Readme), мВ
constexpr uint32_t SUPPLY_CHECK_MS   = 60000;   // период замера напряжения блока, мс
constexpr uint32_t LED_SUPPLY_FAULT_MS = 150;   // полупериод мигания индикатора при обрыве D1, мс

// --- Ночник (сборка rp2040zero_night, Readme.md «Ночник») ---
constexpr uint8_t  LAMP_FULL_PCT       = 100;   // лампа 230 В: «включена» = 100 %
constexpr uint8_t  NIGHT_LEVELS[]      = {5, 10, 20, 35, 60};     // яркость уровней по умолчанию (настройка по USB - pct): заполнение ШИМ = % мощности COB 2 Вт (100 % - слишком ярко, пользователь)
constexpr uint8_t  NIGHT_LEVEL_COUNT   = sizeof(NIGHT_LEVELS);    // число уровней яркости
constexpr uint8_t  NIGHT_DEFAULT_LEVEL = 2;     // уровень после первой калибровки (индекс в NIGHT_LEVELS: 20 %)
constexpr uint32_t NIGHT_PWM_HZ        = 2000;  // частота ШИМ, Гц: без мерцания; затвор P75N02LDG успевает
constexpr uint32_t NIGHT_PWM_RANGE     = 100;   // шкала ШИМ: значение = проценты
constexpr uint32_t NIGHT_PREVIEW_MS    = 3000;  // показ нового уровня, если ночник не горел, мс
// Перегрев: встроенный термодатчик RP2040 + поправка (на железе подобрать) - снизить яркость
constexpr int16_t  TEMP_VBE27_MV       = 706;   // Vbe термодатчика при 27 °C, мВ (п. 4.9.5)
constexpr int16_t  TEMP_SLOPE_UV       = 1721;  // наклон Vbe, мкВ/°C (со знаком минус, п. 4.9.5)
constexpr int16_t  TEMP_REF_C          = 27;    // температура опорной точки Vbe, °C
constexpr int16_t  NIGHT_TEMP_OFFSET_C = 0;     // поправка «кристалл -> воздух у COB», °C (подобрать замером)
constexpr int16_t  NIGHT_TEMP_MAX_C    = 55;    // порог перегрева по умолчанию (настройка по USB - tmax): HLK-5LD05 допускает до 60 °C
constexpr int16_t  NIGHT_TMAX_MIN_C    = 30;    // порог перегрева из консоли: не ниже, °C
constexpr int16_t  NIGHT_TMAX_MAX_C    = 70;    // порог перегрева из консоли: не выше, °C
constexpr int16_t  NIGHT_TEMP_HYST_C   = 5;     // поднимать яркость, когда остынет на столько ниже порога, °C
constexpr uint8_t  NIGHT_DERATE_NUM    = 2;     // перегрев: яркость x NUM / DEN за шаг (на 1/3 меньше, задал пользователь),
constexpr uint8_t  NIGHT_DERATE_DEN    = 3;     //   не ниже уровня 0; остыл - обратно x DEN / NUM за шаг
constexpr uint32_t NIGHT_TEMP_STEP_MS  = 120000;    // шаги яркости по температуре - не чаще, мс: нагрев COB - минуты (оценка)
constexpr uint32_t NIGHT_TEMP_CHECK_MS = 10000; // период замера температуры, мс
