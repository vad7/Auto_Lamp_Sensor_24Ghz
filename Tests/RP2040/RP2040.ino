// Тест минимального потребления Waveshare RP2040-Zero и тока WS2812B.
// Кнопка BOOT переключает режимы по кругу: выкл -> 50 % -> 100 % -> выкл ...
// Цвет - около 3000 K (тёплый белый). Ток платы мерить амперметром в цепи питания.
//
// Минимальное потребление:
//  - системная частота - прямо от кварца 12 МГц, оба PLL выключены (USB и АЦП без 48 МГц не работают);
//  - напряжение ядра 1,05 В - нижняя граница допустимого DVDD 1,05-1,16 В (даташит RP2040, п. 5.5);
//  - USB отключён флагом PIO_FRAMEWORK_ARDUINO_NO_USB (platformio.ini) - прошивка только через BOOT + RESET;
//  - между опросами кнопки - состояние SLEEP (п. 2.11.2): глубокий сон ядра, такты всего, кроме таймера
//    и сторожевого таймера (тик 1 мкс для таймера), остановлены регистрами SLEEP_EN0/1.
//
// Сборка: PlatformIO (platformio.ini рядом), ядро arduino-pico (Earle Philhower), плата waveshare_rp2040_zero;
// библиотека Adafruit NeoPixel.
// Кнопка BOOT не на GPIO (по схеме - линия QSPI_SS_N флеш-памяти), читается объектом BOOTSEL ядра.

#include <Adafruit_NeoPixel.h>
#include <hardware/clocks.h>
#include <hardware/pll.h>
#include <hardware/vreg.h>
#include <hardware/structs/scb.h>
#include <pico/time.h>

// --- Железо (Docs/RP2040-Zero_Schematic.pdf) ---
constexpr uint8_t  LED_PIN   = 16;   // GP16 - вход DIN светодиода WS2812B (L1 на схеме)
constexpr uint16_t LED_COUNT = 1;    // светодиодов на плате, шт.
constexpr uint32_t XOSC_FREQ_HZ = 12000000;  // кварц платы, Гц (X1 12 МГц на схеме)
constexpr vreg_voltage CORE_VOLTAGE = VREG_VOLTAGE_1_05;  // напряжение ядра DVDD (минимум допустимого)
// Такты, работающие в SLEEP: только таймер и сторожевой таймер (SLEEP_EN1, табл. 250 даташита)
constexpr uint32_t SLEEP_EN0_MASK = 0;
constexpr uint32_t SLEEP_EN1_MASK = CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_WATCHDOG_BITS;

// --- Цвет 3000 K: приближение излучения чёрного тела в sRGB (таблица М. Чарити, 3000 K = #FFB46B);
// значения sRGB с гаммой - перед ШИМ переводятся в линейные через gamma8() библиотеки;
// у WS2812B свои первичные цвета, поэтому оттенок примерный ---
constexpr uint8_t COLOR_R = 255;     // красный при 100 %, 0..255 (sRGB)
constexpr uint8_t COLOR_G = 180;     // зелёный при 100 %, 0..255 (sRGB)
constexpr uint8_t COLOR_B = 107;     // синий при 100 %, 0..255 (sRGB)

// --- Режимы ---
constexpr uint8_t MODE_LEVELS_PCT[] = {0, 50, 100};  // яркость режимов, % заполнения ШИМ (линейно)
constexpr uint8_t MODE_COUNT = sizeof(MODE_LEVELS_PCT) / sizeof(MODE_LEVELS_PCT[0]);  // число режимов

// --- Кнопка ---
constexpr uint32_t BUTTON_POLL_MS     = 20;  // период опроса BOOTSEL, мс (между опросами - сон)
constexpr uint8_t  BUTTON_STABLE_POLL = 3;   // опросов подряд в одном состоянии для подавления дребезга

static Adafruit_NeoPixel s_led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // драйвер WS2812B (порядок GRB)
static uint8_t s_mode = 0;           // текущий режим, индекс в MODE_LEVELS_PCT
static bool    s_btnStable = false;  // подтверждённое состояние кнопки: true - нажата
static uint8_t s_btnCount = 0;       // опросов подряд с состоянием, отличным от s_btnStable

// Масштабирует канал на процент яркости.
static uint8_t scale(uint8_t value, uint8_t pct) {
    return (uint8_t)((uint16_t)value * pct / 100);
}

// Применяет режим s_mode.
static void applyMode() {
    const uint8_t pct = MODE_LEVELS_PCT[s_mode];
    s_led.setPixelColor(0, s_led.Color(scale(Adafruit_NeoPixel::gamma8(COLOR_R), pct),
                                       scale(Adafruit_NeoPixel::gamma8(COLOR_G), pct),
                                       scale(Adafruit_NeoPixel::gamma8(COLOR_B), pct)));
    s_led.show();
    // show() только кладёт данные в очередь PIO; до сна (PIO без тактов) дождаться передачи и защёлки 300 мкс
    while (!s_led.canShow()) {
    }
}

// Сон в состоянии SLEEP до срабатывания будильника таймера через ms миллисекунд.
static void sleepMs(uint32_t ms) {
    scb_hw->scr |= M0PLUS_SCR_SLEEPDEEP_BITS;    // WFE -> глубокий сон: такты по маске SLEEP_EN
    best_effort_wfe_or_timeout(make_timeout_time_ms(ms));
    scb_hw->scr &= ~M0PLUS_SCR_SLEEPDEEP_BITS;   // иначе WFE в коде ядра и библиотек тоже усыпит периферию
}

// Системная и периферийная частоты - от кварца, PLL и неиспользуемые такты - выключить.
static void clocksToXosc() {
    clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, XOSC_FREQ_HZ, XOSC_FREQ_HZ);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, XOSC_FREQ_HZ, XOSC_FREQ_HZ);
    clock_stop(clk_usb);
    clock_stop(clk_adc);
    clock_stop(clk_rtc);
    pll_deinit(pll_sys);
    pll_deinit(pll_usb);
    vreg_set_voltage(CORE_VOLTAGE);   // после снижения частоты: на 12 МГц хватает нижней границы
    clocks_hw->sleep_en0 = SLEEP_EN0_MASK;
    clocks_hw->sleep_en1 = SLEEP_EN1_MASK;
}

void setup() {
    clocksToXosc();   // до s_led.begin(): драйвер считает тайминги PIO от текущей clk_sys
    s_led.begin();
    applyMode();      // режим 0 - явно погасить светодиод при старте
}

void loop() {
    sleepMs(BUTTON_POLL_MS);   // сон до следующего опроса кнопки

    const bool pressed = BOOTSEL;
    if (pressed == s_btnStable) {
        s_btnCount = 0;
        return;
    }
    if (++s_btnCount < BUTTON_STABLE_POLL) {
        return;
    }
    s_btnStable = pressed;
    s_btnCount = 0;

    if (s_btnStable) {   // переключение по нажатию
        s_mode = (s_mode + 1) % MODE_COUNT;
        applyMode();
    }
}
