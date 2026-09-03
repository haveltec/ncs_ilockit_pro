#include "power_debug.h"

#if POWER_DEBUG_ENABLED

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <soc.h>
#include <nrfx.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_timer.h>

#if defined(NRF_CRACEN)
#include <hal/nrf_cracen.h>
#endif

LOG_MODULE_REGISTER(power_debug, LOG_LEVEL_INF);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

/* ------------------------------------------------------------------ */
/* Peripherie                                                          */
/* ------------------------------------------------------------------ */

/**@brief Ein ENABLE-Register mit Klartext-Bewertung ausgeben.
 *
 * Bei allen betroffenen Bloecken bedeutet 0 "abgeschaltet". Ein Wert != 0
 * im Ruhezustand ist genau das, wonach wir suchen.
 */
static void enable_reg_log(const char *name, uint32_t value)
{
    LOG_INF("  %-10s ENABLE = 0x%08X  %s", name, value, value ? "<== AKTIV" : "aus");
}

/**@brief Prueft, ob ein TIMER laeuft, indem der Zaehler zweimal gelesen wird.
 *
 * Ein TIMER hat kein ENABLE-Register. Steht der Zaehler, ist der Timer
 * gestoppt; laeuft er, haelt er den Peripherie-Takt und kostet Strom.
 */
static void timer_running_log(const char *name, NRF_TIMER_Type *p_reg)
{
    uint32_t first;
    uint32_t second;

    nrf_timer_task_trigger(p_reg, NRF_TIMER_TASK_CAPTURE5);
    first = nrf_timer_cc_get(p_reg, NRF_TIMER_CC_CHANNEL5);

    k_busy_wait(200);

    nrf_timer_task_trigger(p_reg, NRF_TIMER_TASK_CAPTURE5);
    second = nrf_timer_cc_get(p_reg, NRF_TIMER_CC_CHANNEL5);

    LOG_INF("  %-10s CC5 %u -> %u  %s", name, first, second,
            (first != second) ? "<== LAEUFT" : "gestoppt");
}

static void dump_peripherals(void)
{
    LOG_INF("--- Peripherie ---");

#if defined(NRF_CRACEN)
    /* Bits: 0 = CRYPTOMASTER, 1 = RNG, 2 = PKE/IKG. Im Leerlauf muss das 0
     * sein - cracen_release() schaltet den Block ab. Bleibt hier etwas
     * stehen, haengt ein PSA-Vorgang (Hash/Cipher/RNG) unbeendet fest. */
    enable_reg_log("CRACEN", nrf_cracen_module_get(NRF_CRACEN));
#endif

#if defined(NRF_SPIM00)
    enable_reg_log("SPIM00", NRF_SPIM00->ENABLE);
#endif
#if defined(NRF_TWIM20)
    enable_reg_log("TWIM20", NRF_TWIM20->ENABLE);
#endif
#if defined(NRF_SAADC)
    enable_reg_log("SAADC", NRF_SAADC->ENABLE);
#endif
#if defined(NRF_PWM20)
    enable_reg_log("PWM20", NRF_PWM20->ENABLE);
#endif
#if defined(NRF_PWM21)
    enable_reg_log("PWM21", NRF_PWM21->ENABLE);
#endif
#if defined(NRF_PWM22)
    enable_reg_log("PWM22", NRF_PWM22->ENABLE);
#endif

    timer_running_log("TIMER20", NRF_TIMER20);
    timer_running_log("TIMER22", NRF_TIMER22);
}

/* ------------------------------------------------------------------ */
/* GPIO                                                                */
/* ------------------------------------------------------------------ */

struct pin_entry {
    const char *name;
    uint32_t    psel;   /* absolute Pin-Nummer: Port * 32 + Pin */
};

#define PIN_USER(prop)  { #prop, NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, prop) }
#define PIN_NODE(label, node, prop) { label, NRF_DT_GPIOS_TO_PSEL(node, prop) }

static const struct pin_entry m_pins[] = {
    PIN_USER(charge_gpios),
    PIN_USER(accel_int1_gpios),
    PIN_USER(nled_r_gpios),
    PIN_USER(nled_g_gpios),
    PIN_USER(nled_b_gpios),
    PIN_USER(piezo_freq_gpios),
    PIN_USER(piezo_en_gpios),
    PIN_USER(piezo_en2_gpios),
    PIN_USER(motor_open_gpios),
    PIN_USER(motor_close_gpios),
    PIN_USER(motor_2_open_gpios),
    PIN_USER(motor_2_close_gpios),
    PIN_USER(n_detect_mot_a_gpios),
    PIN_USER(n_detect_mot_b_gpios),
    PIN_USER(motor_sleep_gpios),
    PIN_USER(hall_open_gpios),
    PIN_USER(hall_close_gpios),
    PIN_USER(gps_enable_gpios),
    PIN_USER(fe_enable_gpios),
    PIN_USER(plug_detection_gpios),
    PIN_NODE("main_button", DT_ALIAS(sw0), gpios),
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(can_controller))
    PIN_NODE("can_int", DT_NODELABEL(can_controller), int_gpios),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(spi00))
    PIN_NODE("can_cs", DT_NODELABEL(spi00), cs_gpios),
#endif
};

static void dump_pins(void)
{
    LOG_INF("--- GPIO (P<port>.<pin>) ---");

    for (uint8_t i = 0; i < ARRAY_SIZE(m_pins); i++)
    {
        uint32_t psel = m_pins[i].psel;

        nrf_gpio_pin_dir_t   dir   = nrf_gpio_pin_dir_get(psel);
        nrf_gpio_pin_input_t input = nrf_gpio_pin_input_get(psel);
        nrf_gpio_pin_pull_t  pull  = nrf_gpio_pin_pull_get(psel);

        const char *dir_str = (dir == NRF_GPIO_PIN_DIR_OUTPUT) ? "OUT" : "in ";
        const char *pull_str = (pull == NRF_GPIO_PIN_PULLUP)   ? "PU"
                             : (pull == NRF_GPIO_PIN_PULLDOWN) ? "PD"
                                                               : "--";

        if (dir == NRF_GPIO_PIN_DIR_OUTPUT)
        {
            LOG_INF("  P%u.%02u %-22s %s  out=%u", psel / 32, psel % 32,
                    m_pins[i].name, dir_str, nrf_gpio_pin_out_read(psel));
        }
        else if (input == NRF_GPIO_PIN_INPUT_DISCONNECT)
        {
            /* Eingangspuffer getrennt - der sichere Ruhezustand. */
            LOG_INF("  P%u.%02u %-22s abgeklemmt", psel / 32, psel % 32,
                    m_pins[i].name);
        }
        else
        {
            /* Verbundener Eingang OHNE Pull ist der Verdachtsfall: liegt der
             * Pegel extern nicht fest, zieht der Eingangspuffer dauerhaft
             * Querstrom. */
            LOG_INF("  P%u.%02u %-22s %s  pull=%s  level=%u  %s",
                    psel / 32, psel % 32, m_pins[i].name, dir_str, pull_str,
                    nrf_gpio_pin_read(psel),
                    (pull == NRF_GPIO_PIN_NOPULL) ? "<== Eingang ohne Pull" : "");
        }
    }
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

void power_debug_dump(void)
{
    LOG_INF("===== power_debug =====");
    dump_peripherals();
    dump_pins();
    LOG_INF("=======================");
}

void power_debug_tick(void)
{
    static int64_t next_dump;

    int64_t now = k_uptime_get();

    if (now < next_dump)
    {
        return;
    }

    next_dump = now + POWER_DEBUG_PERIOD_MS;
    power_debug_dump();
}

#endif /* POWER_DEBUG_ENABLED */
