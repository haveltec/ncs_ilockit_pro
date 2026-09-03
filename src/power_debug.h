/**
 * @file power_debug.h
 * @brief Diagnose fuer die Suche nach Ruhestrom-Verursachern.
 *
 * Liest im Leerlauf die ENABLE-Register der Peripherie und die Pin-
 * Konfiguration aller genutzten GPIOs aus und gibt sie ueber das Log aus.
 * Damit laesst sich ohne Raten feststellen, welcher Block im Ruhezustand
 * noch aktiv ist bzw. welcher Pin offen (floating) am Eingangspuffer haengt.
 *
 * Nur ein Diagnose-Werkzeug - vor dem Serienstand wieder entfernen bzw.
 * POWER_DEBUG_ENABLED auf 0 setzen.
 */

#ifndef POWER_DEBUG_H__
#define POWER_DEBUG_H__

#include <stdint.h>

/* Auf 0 setzen, um die komplette Diagnose aus dem Build zu nehmen. */
#define POWER_DEBUG_ENABLED     1

/* Abstand zwischen zwei zyklischen Ausgaben in Millisekunden. */
#define POWER_DEBUG_PERIOD_MS   10000

/* Ruhestrom-Test: Auf 1 setzen, dann parkt main() sofort in einer Schlaf-
 * schleife - keine App-Initialisierung, kein Bluetooth, kein Watchdog.
 * Es laufen nur die Zephyr-Treiber-Inits, die vor main() ausgefuehrt werden.
 *
 * Damit laesst sich der Grundstrom von SoC und Platine vom Beitrag der
 * Anwendung trennen. Zusaetzlich sollte fuer diesen Test can_controller im
 * Board-DTS auf "disabled" stehen, sonst laesst der Zephyr-CAN-Treiber den
 * MCP2518FD nach seinem Init wach zurueck (ili_can_init() laeuft ja nicht).
 *
 * ACHTUNG: Mit diesem Schalter tut das Geraet nichts mehr - reiner Messstand. */
#define POWER_DEBUG_IDLE_ONLY   0

#if POWER_DEBUG_ENABLED

/**@brief Gibt den aktuellen Zustand von Peripherie und Pins einmalig aus. */
void power_debug_dump(void);

/**@brief Aus der Hauptschleife aufrufen - gibt hoechstens alle
 *        POWER_DEBUG_PERIOD_MS Millisekunden aus. */
void power_debug_tick(void);

#else

static inline void power_debug_dump(void) {}
static inline void power_debug_tick(void) {}

#endif /* POWER_DEBUG_ENABLED */

#endif /* POWER_DEBUG_H__ */
