/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/logging/log.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/settings/settings.h>
#include <zephyr/drivers/pwm.h>

#include <psa/crypto.h>
#include <psa/crypto_extra.h>
#include <zephyr/sys/crc.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/zms.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/retention/retention.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/pm/device.h>
#include <zephyr/usb/usb_device.h>

#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

//#include <fmna.h>
// MCU Management Callbacks
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>

// PSA Secure Storage
#include <zephyr/psa/key_ids.h>

// I LOCK IT Bibliotheken
#include "ili.h"
#include "ili_parser.h"
#include "ili_uicr.h"
#include "ili_button.h"
#include "ili_motorcontroller.h"
#include "ili_battery.h"
#include "ili_accelerometer.h"
#include "ili_client.h"
#include "ili_piezo.h"
#include "ili_led.h"
#include "ili_can.h"
#include "commands.h"
#include "defines.h"

//#define TODO
// Helfer für spätere Anpassungen
#ifdef TODO 
#define TD(log) log
#else
#define TD(log)
#endif


// Logging Modul registrieren
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);


//////////////////////////////////////////////////////////////
//            App-spezifische Konfigurationen               //
//////////////////////////////////////////////////////////////

// Anzahl der maximal anlernbaren Smartphones
#define MAX_PEER_COUNT			8
#define BLE_CONN_HANDLE_INVALID 0xFFFF  /**< Invalid Connection Handle. */
#define BLE_CONN_HANDLE_ALL     0xFFFE  /**< Applies to all Connection Handles. */
#define APP_SUCCESS				0
#define APP_ERROR				-1
#define AUTH_ID_NONE            0xFF

//////////////////////////////////////////////////////////////
//                    Typdefinitionen                       //
//////////////////////////////////////////////////////////////

// https://github.com/too1/ncs-whack-a-mole/blob/master/central/src/app_bt.c#L51-L57

typedef struct
{
    uint8_t  hw_id[HW_ID_LEN];
    uint8_t  auth_id;
    bool     is_used;
} peer_data_t;

struct peer_context_s
{
	bool used;
	struct bt_conn* conn;
	uint8_t index;
	peer_data_t* peer_data;
    uint8_t ltk[LTK_LEN];
};

typedef struct peer_context_s peer_context_t;

typedef struct
{
    bool    armed;
    bool    silent;
    bool    prealarm;
    uint8_t threshold;
    uint8_t ble;
} alarm_settings_t;

typedef struct
{
    bool    open_active;
    bool    close_active;
    uint8_t threshold_open;
    uint8_t threshold_close;
} auto_settings_t;

typedef struct
{
    uint8_t mode;
    uint8_t ble;
} sound_settings_t;

#define RESERVED_BYTES  31
typedef struct
{
    alarm_settings_t alarm;
    sound_settings_t sound;
    auto_settings_t auto_lock[MAX_PEER_COUNT];
    uint8_t colorcode_flash[3];
    uint8_t sharecode_flash[3];
    uint8_t theft_mode;
    uint8_t reserved[RESERVED_BYTES];
} settings_t;

typedef enum
{
    UNAUTHORISED = 0,           // Unautorisiert
    CHALLENGE_REQUESTED,        // Challenge angefragt
    TOKEN_CORRECT,              // Korrektes Autorisierungs-Token empfangen
    TOKEN_CHALLENGE_SENT,       // Challenge nach Token geschickt
    AUTH_DATA_CORRECT,	        // Korrekte Autentifizierungs-Daten empfangen
    CHALL_RESP_CORRECT,         // Korrekte Challenge-Response empfangen
    AUTH_ID_CORRECT,            // Korrekte Autentifizierungs-ID empfangen
    PEER_SAVED,                 // Peer-Daten gespeichert
    IV_SENT,                    // Initialisierungsvektor verschickt
    LTK_SENT,                   // LTK-Seed verschickt
    AUTHORISED                  // Autorisiert
} auth_state_t;

typedef enum
{
    LED_STATIC = 0,
    LED_ERROR,
    LED_DISCONNECT,
    LED_BONDING,
    LED_LOCKING,
    LED_UNLOCKING,
    LED_CHARGING,
    LED_ALARM,
    LED_DOUBLE_TAP,
    LED_SIGNAL
} led_fading_t;

/** @brief Struktur für Statusmeldungen*/
typedef struct
{
    uint8_t     status_code;
    uint16_t    conn_handle;
} status_t;

/**@brief Name des Centrals mit variabler Länge*/
typedef struct
{
	uint8_t     * p_data;    /**< Pointer to data. */
	uint16_t      data_len;  /**< Length of data. */
} central_data_t;


//////////////////////////////////////////////////////////////
//                Funktionsdeklatationen                    //
//////////////////////////////////////////////////////////////
static void gdio_data_received(uint16_t conn_handle);               // Funktion für den Empfang von Daten über die GDIO-Charakteristik
static void usdio_data_received(uint16_t conn_handle);              // Funktion für den Empfang von Daten über die USDIO-Charakteristik
static void usdio_send_data(uint16_t conn_handle, ili_usdio_message_t* message_out);
static void gdio_send_data(uint16_t conn_handle, ili_gdio_message_t* message_out);
static void send_status(uint16_t conn_handle, uint8_t status);
static void advertising_start();
static void advertising_stop();
static void all_sounds_stop();                                      // Funktion zum Beenden des Alarmtons
static void set_motion_detection(bool enable);                      // Funktion zum Öffnen/Schließen des Schlosses
static void alarmcheck_start();                                     // Alarmprüfung starten
static void alarmcheck_stop(void);                                  // Alarmprüfung stoppen
static void alarmsound_start(void);                                 // Funktion zum Abspielen des Alarmtons
static void prealarm_start(void);                                   // Funktion zum Abspielen des Voralarms
static void signalsound_start(void);                                // funktion zum Abspielen des Signaltons
static void beep_start(uint8_t beep_type);                          // Funktion zum Starten eines Pieptons
static bool authorised_user_is_nearby();                            // Gibt an, ob sich ein autorisierter Nutzer in der Nähe befindet
static bool authorised_user_connected();                            // Gibt an, ob ein autorisierter Nutzer verbunden ist
static void led_faded(uint8_t color, led_fading_t fading_type);     // Funktion für das Faden der LEDs
static void led(uint8_t color);                                     // Funktion für das Aktivieren der LEDs
static void led_timed(uint8_t color, led_fading_t fading_type);     // Funktion für das zeitlich begrenzte Aktivieren von LEDs
static void led_off();                                              // Funktion zum Deaktivieren der LED
static void set_colorcodes();                                       // Funktion zum Setzen der Farbcodes
static void rssi_start();                                           // RSSI-Auswertung für ein Peer starten
static void rssi_stop();                                            // RSSI-Auswertung stoppen
static void app_gdio_data_rx_cb(struct bt_conn* conn, uint8_t* data, uint8_t len);  // Callback für Empfang von Daten per GDIO
static void app_usdio_data_rx_cb(struct bt_conn* conn, uint8_t* data, uint8_t len); // Callback für Empfang von Daten per USDIO
void app_gdio_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err);   // Callback für Empfang von ACK per GDIO
void app_usdio_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err);  // Callback für Empfang von ACK per USDIO
void get_rssi();                                                    // Thread-Funktion zum Auslesen der RSSI-Werte
static void discovery_completed_cb(struct bt_gatt_dm *dm,
				   void *context);                                  // Callback wenn die Service-Discovery abgeschlossen ist
static void discovery_service_not_found_cb(struct bt_conn *conn,
					   void *context);                              // Callback, falls der Service nicht gefunden wurde
static void discovery_error_found_cb(struct bt_conn *conn,
				     int err,
				     void *context);                                // Callback, falls ein Fehler bei der Discovery auftrat
static void ble_mode_data_received(struct bt_ili_client *ili, const uint8_t *const data, uint16_t len);
static void ble_mode_data_sent(struct bt_ili_client *ili,uint8_t err, const uint8_t *const data, uint16_t len);
static void ble_mode_unsubscribed(struct bt_ili_client *ili);
static void auth_cancel(struct bt_conn *conn);
static void pairing_confirm(struct bt_conn *conn);
static void pairing_complete(struct bt_conn *conn, bool bonded);
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason);
static int  scan_start(void);
static void setup_accept_list_cb(const struct bt_bond_info *info, void *user_data);
//static void neo_device_name_set(bool force);
//static void fmna_enable_work_handle(struct k_work *item);
//static void fmna_disable_work_handle(struct k_work *item);
//static void fmna_motion_detect_handle(struct k_work *item);
static void abort_colorcode_input();
static void start_adv_handler(struct k_work* work);
static void app_movement_timeout_handler(struct k_work* work);                                         // UART initialisieren
static void uart_evt_handler(const struct device* dev,
                             struct uart_event* evt, void* user_data);              // Callback für die Ereignisse des UART-Treibers
static void gsm_send(uint8_t command);                              // Funktion zum Senden einer GSM-Nachricht
static void uart_gsm_send_ack(uint8_t command);                     // Funktion zum Senden eines Acknowledges
static void gps_on();
static void gps_off();
//static void start_fmna_pairing_handler(struct k_work* work);

//////////////////////////////////////////////////////////////
//                    Timeout Handler                       //
//////////////////////////////////////////////////////////////
static void gdio_ind_ack_timeout_handler(struct k_timer *timer);
static void usdio_ind_ack_timeout_handler(struct k_timer *timer);
static void bond_timeout_handler(struct k_timer *timer);
static void alarm_restart_timeout_handler(struct k_timer *timer);
static void reset_prealarm_timeout_handler(struct k_timer *timer);
static void colorcode_input_timeout_handler(struct k_timer *timer);
static void colorcode_timeout_handler(struct k_timer *timer);
static void service_code_reset_timeout_handler(struct k_timer *timer);
static void reset_wrong_colorcode_input_timeout_handler(struct k_timer* timer);
static void alarm_timeout_handler(struct k_timer* timer);
static void check_alarm_timeout_handler(struct k_timer* timer);
static void six_min_timeout_handler(struct k_timer* timer);
static void plug_reactivation_timeout_handler(struct k_timer* timer);
static void bond_allowed_timeout_handler(struct k_timer* timer);
static void auth_timeout_handler(struct k_timer* timer);
static void gsm_send_timeout_handler(struct k_timer* timer);

//static void abort_fmna_user_pairing_timeout_handler(struct k_timer* timer);

//////////////////////////////////////////////////////////////
//                          BLE                             //
//////////////////////////////////////////////////////////////
#define BLE_ADV_CONN_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, APP_ADV_INTERVAL_MIN, APP_ADV_INTERVAL_MAX,  \
			NULL)

// ---------------------------------------------------------------------------
// Test-Schalter fuer die Advertising-Fehlersuche
//   ADV_TEST_LEGACY = 1 -> einfaches (Legacy) Advertising via bt_le_adv_start()
//   ADV_TEST_LEGACY = 0 -> Extended Advertising via bt_le_ext_adv_*()
// ADV_TEST_NAME ist der fest vorgegebene Advertising-Name (statt UICR-Name).
// ---------------------------------------------------------------------------
#define ADV_TEST_LEGACY     1
#define ADV_TEST_NAME       "ILI-PRO-TEST"

#if ADV_TEST_LEGACY
// Advertising-Daten: Flags + vollstaendiger Geraetename
static const struct bt_data m_adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, ADV_TEST_NAME, sizeof(ADV_TEST_NAME) - 1)
};
#else
static struct bt_le_ext_adv *m_ili_pro_adv_set;
#endif

static struct ili_service_cb ili_callbacks = {
	.gdio_data_rx_cb    = app_gdio_data_rx_cb,
	.usdio_data_rx_cb   = app_usdio_data_rx_cb,
    .gdio_indicate_cb   = app_gdio_indicate_cb,
    .usdio_indicate_cb  = app_usdio_indicate_cb
};
#define CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT 3
static bool             m_advertising_is_active = false;                                    // Flag, das angibt, ob das Advertisment momentan läuft
static bool             m_bonding_mode_active = false;                                      // Flag für den aktiven Bondingmodus
static bool             m_gdio_indication_ack = false;                                      // Flag, das angibt ob ein Acknowledge für eine Indication eingetroffen ist
static bool             m_usdio_indication_ack = false;                                     // Flag, das angibt ob ein Acknowledge für eine Indication eingetroffen ist
static uint8_t          m_num_app_bonds = 0;                                                // Anzahl der gekoppelten Smartphones
static uint8_t          m_num_fob_bonds = 0;                                                // Anzahl der gekoppelten HS+
static peer_context_t   m_connected_peer[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT] = {0};        // Gibt an, mit welchem Peer eine Verbindung besteht
static uint8_t          m_rssi_values[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT][RSSI_AVG_COUNT]; // Liste mit RSSI-Werten für die jeweiligen Peers
static uint8_t          m_rssi_pos[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];                    // Liste mit Indizes der RSSI-Berechnung
static uint16_t         m_rssi_sum[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];                    // Liste mit Summen der RSSI-Berechnung
static uint8_t          m_rssi_avg[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];                    // Liste der durchschn. RSSI-Werte
static uint8_t          m_rssi_user_state[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];             // Liste der aktuellen Zustände der Nutzer
static bool             m_rssi_active = false;                                              // Flag für RSSI-Auswertung
static uint8_t			m_peripheral_conn_count = 0;										// Variable für die Anzahl aktueller Verbindungen

static k_tid_t          m_rssi_thread_id;                                                   // ID zur Steuerung des Threads
struct k_thread         m_rssi_thread;                                                      // Thread Struktur
K_THREAD_STACK_DEFINE(m_rssi_stack, RSSI_STACKSIZE);                                        // Stack für RSSI-Thread
K_MUTEX_DEFINE(m_rssi_mutex);                                                               // Mutex für Thread-Synchronisation
static uint8_t          m_disconnect_device = 0;                                            // Speichert die Indizes der Geräte, deren Verbindung getrennt werden soll
static bool             m_stop_rssi = false;
static uint8_t          m_relock_state = RELOCK_INACTIVE;                                   // Flag für das Schließen nach automatischer Öffnung

//////////////////////////////////////////////////////////////
//                   App-Kommunikation                      //
//////////////////////////////////////////////////////////////
static __ALIGN(4) uint8_t           m_stk[LTK_LEN];                                               // Short Term Key zur Schlüsselübertragung	
static uint8_t                      m_ltk[LTK_LEN];                                                 // Long Term Key
static uint8_t                      m_app_msg_buffer[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT][128];   // Puffer für BLE-Nachricht     
static uint8_t                      m_app_msg_buffer_length[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT]; // Länge der BLE-Nachricht
static __ALIGN(4) peer_data_t       m_peer_data[MAX_PEER_COUNT];                                // Peer ID und LTK der verbundenen Peers (maxmal 8)

static auth_state_t 		        m_auth_state[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];            // Status der Authentifizierung
static uint16_t                     m_gdio_data_received = 0;                                   // Flag, das angibt dass Daten über die GDIO-Charakteristik empfangen wurden
static uint16_t                     m_usdio_data_received = 0;                                  // Flag, das angibt dass Daten über die USDIO-Charakteristik empfangen wurden
static ili_gdio_message_t           m_gdio_message_in[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];       // Nachricht, die von der App über die GDIO-Charakteristik empfangen wurde
static ili_usdio_message_t          m_usdio_message_in[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];      // Nachricht, die von der App über die USDIO-Charakteristik empfangen wurde
static psa_hash_operation_t 		m_sha256 = {0};													// Variable für die Hash-Berechnung
static size_t                       m_sha256_len = SHA256_LENGTH;                               // Länge des Hashwerts
static uint8_t                      m_nonce[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT][SHA256_LENGTH];  // Zufallszahl
static uint16_t                     m_lock_counter[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];          // Zähler für Schließvorgänge
static uint16_t                     m_peer_started_bonding = BLE_CONN_HANDLE_INVALID;           // Gibt an, welcher Peer momentan ein Bonding erstellen will
static bool                         m_reset_incompatible_app = false;                           // Flag, dass angibt ob das Schloss durch ein falsches Config-Paket in den Werkszustand gesetzt werden kann


#define STATUS_QUEUE_LENGTH 16
K_MSGQ_DEFINE(m_status_msgq, sizeof(status_t), STATUS_QUEUE_LENGTH, 4);                         // Queue für Statusmeldungen

//////////////////////////////////////////////////////////////
//               Handsender-Kommunikation                   //
//////////////////////////////////////////////////////////////
static bool                         m_bonded_fob_connected = false;                     // Flag, das angibt ob ein gekoppelter Handsender verbunden ist
static struct bt_conn*              m_fob_conn = NULL;                                  // Struktur für Handsender-Verbindung 
static const char                   m_keyfob_name[8] = "KEYFOB-";                       // Namensteil für Filterung
static struct bt_ili_client         m_ili_fob;                                          // Client Struktur für den ILI Handsender Service
static uint8_t                      m_fob_status = 0;                                   // Variable für den Status, der an den Handsender gesendet werden soll

static struct bt_gatt_dm_cb discovery_cb = {                                            // DB Discovery Callbacks
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};
static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
    .passkey_display = NULL,
    .passkey_confirm = NULL,
	.pairing_confirm = NULL
};
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

const struct bt_le_scan_param SCAN_PARAM_WHITELIST = {
	.interval = SCAN_INTERVAL,
	.interval_coded = 0,
	.options = BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST,
	.timeout = 0,
	.type = BT_HCI_LE_SCAN_PASSIVE,
	.window = SCAN_WINDOW,
	.window_coded = 0,
};
const struct bt_le_scan_param SCAN_PARAM_BONDING = {
	.interval = SCAN_INTERVAL_BONDING,
	.interval_coded = 0,
	.options = BT_LE_SCAN_OPT_NONE,
	.timeout = 0,
	.type = BT_HCI_LE_SCAN_PASSIVE,
	.window = SCAN_WINDOW_BONDING,
	.window_coded = 0,
};
const struct bt_le_conn_param COMM_PARAM = {
	.interval_min = CONN_INT_MIN,
	.interval_max = CONN_INT_MAX,
	.latency = 0,
	.timeout = BT_GAP_MS_TO_CONN_TIMEOUT(3000),
};
const struct bt_scan_init_param SCAN_INIT_PARAMS = {
	.connect_if_match = false,
	.conn_param = &COMM_PARAM,
	.scan_param = &SCAN_PARAM_WHITELIST,
};

//////////////////////////////////////////////////////////////
//                    Schlosszustand                        //
//////////////////////////////////////////////////////////////
static bool                     m_factory_condition = true;                                 // Gibt an, ob sich das Schloss im Werkszustand befindet
static uint8_t                  m_current_locking_state;                                    
static uint8_t                  m_current_locking_state_chain;                              // 0x0B = Geöffnet, 0x02 = Geschlossen, 0x0F = Nicht definiert (z.B. blockiert);
static bool                     m_bonding_allowed = false;                                  // Flag, das angibt ob eine neue Kopplung möglich ist
static __ALIGN(4) settings_t    m_settings;                                                 // Schloss-Einstellungen
ili_uicr_data_t                 m_uicr_data;                                                // Schlossdaten aus dem UICR
static bool                     m_dnd_mode_active = false;                                  // Flag für den NIcht-Stören Modus
static bool                     m_restart_lock = false;                                     // Gibt an, ob das Schloss neugestartet werden soll
static bool                     m_bootloader_prepare = false;                               // Flag, das angibt, ob der BL betreten werden soll
static bool                     m_acc_check_passed = true;                                  // Flag, das angibt, ob der Selbsttest des Beschleunigungssensors erfolgreich war
static bool                     m_chain_is_present = false;                                 // Flag, das angibt, ob die Einsteckkette im Schloss steckt oder nicht
static bool                     m_charge_active = false;                                    // Flag, das angibt, ob das Schloss geladen wird
static uint8_t                  m_service_code_state = SERVICE_CODE_INACTIVE;               // Flag, das angibt, ob die Eingabe des Service-Farbcodes erlaubt ist
static uint8_t                  m_batt_timeout_counter = 0;                                 // Zähler um den Batteriestand alle 24h zu messen
static bool                     m_play_batt_warning = false;                                // Flag zum Abspielen eines Warntons nach einer Batteriemessung
static uint8_t                  m_bl_version;                                               // Bootloader Version an Adresse im RAM;
static bool                     m_start_factory_reset = false;                              // Gibt an, ob er Werkszustand hergestellt werden soll
const struct device*            m_ram = DEVICE_DT_GET(DT_NODELABEL(retention0));            // Region im RAM der bei Reset erhalten bleibt
static bool                     m_chain_temp_disabled = false;                              // Gibt an, ob die Kette temporär deaktiviert wurde
static bool                     m_restart_allowed = false;                                  // Gibt an, ob das Schloss per Ladekabel neu gestartet werden kann

//////////////////////////////////////////////////////////////
//               Alarm / Signalton                          //
//////////////////////////////////////////////////////////////
uint8_t     m_alarmcounter = 0;                 // Zähler für Bewegungssamples zum Ermitteln eines Alarms
uint8_t     m_collected_debounce_samples = 0;   // Zähler für Samples, die während des Debounce gesammelt wurden
bool        m_alarmsound_active = false;        // Gibt an, ob der Alarm aktiv ist
bool        m_prealarm_active = false;          // Gibt an, ob der Voralarm aktiv ist
bool        m_prealarm_fired = false;           // Gibt an, ob ein Warnton bereits abgespielt wurde
bool        m_signalsound_active = false;       // Gibt an, ob ein Signalton aktiv ist

//////////////////////////////////////////////////////////////
//                        GPIOs                             //
//////////////////////////////////////////////////////////////

//https://docs.zephyrproject.org/latest/build/dts/zephyr-user-node.html
//https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec pin_charge = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, charge_gpios);
static const struct gpio_dt_spec pin_accel_int1 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, accel_int1_gpios);
static const struct gpio_dt_spec pin_gps_enable = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, gps_enable_gpios);
static const struct gpio_dt_spec pin_gps_button = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, fe_enable_gpios);


static struct gpio_callback accel_int1_cb_data;
static struct gpio_callback charge_cb_data;

uint32_t    m_triggered_pins = 0;               // Enthält die Pins, die im GPIOTE_Handler erkannt wurden
uint16_t    m_button_counter = 0;               // Anzahl der Ticks beim Tastendruck

//////////////////////////////////////////////////////////////
//                      LED-Farben                          //
//////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
//                        Flash                             //
//////////////////////////////////////////////////////////////
#define ZMS_PARTITION		        storage_partition
#define ZMS_PARTITION_DEVICE	    FIXED_PARTITION_DEVICE(ZMS_PARTITION)
#define ZMS_PARTITION_OFFSET	    FIXED_PARTITION_OFFSET(ZMS_PARTITION)
static struct zms_fs                m_filesys;
static bool volatile                m_zms_initialized;                      // Flag zum Prüfen der ZMS-Initialisierung
static uint8_t                      m_update_peer_auth_id = AUTH_ID_NONE;   // Flag für das Speichern der Peer-Daten
static bool                         m_update_settings = false;              // Flag für das Speichern der Einstellungen

// Partition Manager
// https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/scripts/partition_manager/partition_manager.html


//////////////////////////////////////////////////////////////
//                       Farbcode                           //
//////////////////////////////////////////////////////////////
static bool         m_write_click_code = false;                 // Flag zum Überprüfen des Schreibvorgangs bei Ersteinrichtung
static uint8_t      m_colorcode_in_index = 0;                   // Zähler für Position in click_code_in
static uint8_t      m_colorcode_in[6];                          // Array für die Eingabe des Farbcodes über den Taster
static uint8_t      m_colorcode[6];                             // Farbcode zur Eingabe über den Taster
static uint8_t      m_sharecode[6];                             // Sharecode zur Eingabe über den Taster
static uint8_t      m_colorcode_service_code_enable[6];         // Farbcode, der eingegeben werden muss um die Eingabe des Service-Codes zu starten 
static uint8_t      m_selected_color = SELECTED_COLOR_INIT_VAL; // Gibt an, welche Farbe aktuell angezeigt wird
static uint8_t      m_colorcode_input_active = false;           // Flag, das angibt ob die Farbcode-Eingabe aktiv ist
static uint8_t      m_wrong_colorcode_attempts = 0;             // Anzahl der falsch eingegebenen Farbcodes
static bool         m_colorcode_input_allowed = true;           // Flag, das angbit, ob die Eingabe des Farbcodes erlaubt ist


#define DEBOUNCE_WINDOW_MS              80
#define SINGLE_TAP_WINDOW_MS            180
#define DOUBLE_TAP_WINDOW_MS            300
#define DOUBLE_TAP_PROOF_TIME_MS        1200
#define TAP_ERROR_THRESHOLD             5
#define NUM_DOUBLE_TAP_WAITING_ERRORS   1

//////////////////////////////////////////////////////////////
//                       Testmodus                          //
//////////////////////////////////////////////////////////////
static uint16_t                 m_needed_tests = TEST_NOT_FOUND;                            // Die notwenigen Tests, die ausgeführt werden sollen
static bool                     m_gsm_test_successfull = false;                             // Flag, das angibt ob der GSM Test erfolgreich war
static bool                     m_test_active = false;                                      // Flag, das angibt, ob momentan ein Test aktiv ist
static uint16_t                 m_actual_test = 0;                                          // Zähler für den aktuell auszuführenden Test
static bool                     m_update_testmode = false;                                  // Flag für das Speichern des Testmode-Zustands
static __ALIGN(4) uint16_t      m_testmode_state;

//////////////////////////////////////////////////////////////
//                       Watchdog                           //
//////////////////////////////////////////////////////////////
const struct device *const  m_watchdog = DEVICE_DT_GET(DT_ALIAS(watchdog0));
int                         m_wdt_channel_id = 0;

//////////////////////////////////////////////////////////////
//                         UART                             //
//////////////////////////////////////////////////////////////


static uint8_t          m_uart_tx_buf[GSM_TX_BUFF_SIZE];            // Sendepuffer, muss bis UART_TX_DONE gültig bleiben
static K_SEM_DEFINE(m_uart_tx_sem, 1, 1);                           // Gibt an, ob der Sendepuffer frei ist

static uint8_t          m_uart_rx_buf[2][GSM_RX_BUFF_SIZE];         // Doppelpuffer für den Empfang per DMA
static uint8_t          m_uart_rx_buf_index = 0;                    // Index des Puffers, der dem Treiber zuletzt übergeben wurde
static uint8_t          m_uart_rx_line[UART_RX_BUF_SIZE];           // Puffer zum Zusammensetzen einer Nachricht (wird im Interrupt gefüllt)
static uint16_t         m_uart_rx_line_len = 0;                     // Anzahl der Zeichen in m_uart_rx_line
static uint8_t          m_uart_rx_message[UART_RX_BUF_SIZE];        // Vollständig empfangene Nachricht für die Auswertung in der Main-Schleife
static uint16_t         m_uart_rx_message_len = 0;                  // Länge der Nachricht in m_uart_rx_message

//////////////////////////////////////////////////////////////
//                     GSM / GPS                            //
//////////////////////////////////////////////////////////////
// UARTE00 ist mit dem GPS-Modul verbunden (TX = P2.08, RX = P2.00, 9600 Baud)
//
// ACHTUNG: UARTE00 und SPIM00 sind derselbe Serial-Block (0x0004A000) - es kann
// immer nur eines von beidem laufen. Ist im Board-DTS spi00 (CAN) aktiv, ist
// uart00 zwangslaeufig deaktiviert und es gibt kein GPS-UART. Damit die
// GPS-Logik trotzdem unveraendert uebersetzt, bleibt m_uart dann NULL; alle
// UART-Zugriffe pruefen darauf.
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(uart00))
const struct    device *const  m_uart = DEVICE_DT_GET(DT_NODELABEL(uart00));
#else
const struct    device *const  m_uart = NULL;
#endif
K_MSGQ_DEFINE(m_gsm_cmd_queue, sizeof(uint8_t), 5, 4);
static bool     m_gps_active = false;           // Gibt an, ob das GPS-Modul aktiv ist
static bool     m_gsm_msg_available = false;    // Gibt an, ob eine komplette Nachricht empfangen wurde
static ili_gsm_message_t    m_gsm_message_in;   // Nachricht, die per UART vom GSM-Modul empfangen wurde
static uint8_t  m_gsm_send_counter = 0;         // Zähler für die Sendeversuche
static bool     m_gsm_send_cmd = false;         // Flag zum Senden von Befehlen in der Hauptschleife
static uint8_t  m_gps_lpw_counter = 0;          // Zähler zum Deaktivieren des GPS-Moduls (nur für Low Power Alarm Tracking Modus)
static uint8_t  m_gps_lpw_timeout = 0;          // Schwelle für den Wechsel in den LowPower Modus (m_gps_lpw_timeout * 6 Min.)
static bool     m_gps_test_active = false;      // Flag, das angibt, ob das GPS-Modul vom Nutzer aktiviert wurde
static uint32_t m_gsm_ack_ref_id = 0;           // Referenz-ID für einen empfangenen Befehl

//////////////////////////////////////////////////////////////
//                     Apple Find-My                        //
//////////////////////////////////////////////////////////////
//static bool m_fmna_location_available = false;
//static bool m_fmna_pairing_mode_active = false;
//static bool m_fmna_paired = false;
//static bool m_fmna_motion_detection_active = false;
//static bool m_fmna_motion_detected = false;
//static bool m_fmna_sound_started = false;
//static bool m_fmna_user_pairing_active = false;
//static sample_t m_last_fmna_sample = {0};

/* Product Data configuration: PD */
//const uint8_t fmna_pp_product_data[] = {
//        0x7c, 0x3e, 0xbd, 0x7d, 0xaf, 0x3e, 0x1d, 0xbe
//};

/* Server encryption key: Q_E */
//const uint8_t fmna_pp_server_encryption_key[] = {
//	0x04, 0x9C, 0xC5, 0xAD, 0xDD, 0xD0, 0x29, 0xB7,
//    0x53, 0x5D, 0x30, 0xE6, 0xE5, 0xD1, 0x6D, 0xB7,
//    0xA8, 0xD2, 0x1B, 0x1B, 0x48, 0xB5, 0x5B, 0x19,
//    0xD5, 0xB1, 0x10, 0xE9, 0x5B, 0xF3, 0x15, 0x45,
//    0xE7, 0x74, 0xCF, 0x51, 0x8D, 0xEB, 0xBE, 0x3C,
//    0x71, 0x68, 0x33, 0xE4, 0x43, 0xF1, 0x14, 0x47,
//    0x6E, 0x5A, 0x4B, 0x05, 0x4E, 0x36, 0x75, 0x07,
//    0x05, 0x6E, 0x39, 0x95, 0xCC, 0x6B, 0x96, 0x90,
//    0x96
//};

/* Server signature verification key: Q_A */
//const uint8_t fmna_pp_server_sig_verification_key[] = {
//	0x04, 0x33, 0x4C, 0x5A, 0x73, 0xFD, 0x61, 0xDF,
//    0x36, 0x43, 0x3F, 0xBC, 0x69, 0x92, 0x36, 0xE3,
//    0x98, 0xE4, 0x94, 0x12, 0xF3, 0xC0, 0xFD, 0xC4,
//    0xE5, 0xDA, 0x0B, 0x41, 0x18, 0x77, 0x95, 0x17,
//    0x08, 0x71, 0x20, 0x88, 0x8E, 0x97, 0x92, 0x37,
//    0x76, 0xBA, 0x48, 0xDC, 0x51, 0x7C, 0x0F, 0xA8,
//    0x7B, 0x9C, 0x62, 0xA9, 0xFE, 0xE9, 0x6B, 0x0F,
//    0x38, 0x40, 0x3F, 0x66, 0x9E, 0x1E, 0x67, 0x55,
//    0x60
//};

//////////////////////////////////////////////////////////////
//                          DFU                             //
//////////////////////////////////////////////////////////////
struct mgmt_callback mcumgr_dfu_cb;
struct mgmt_callback mcumgr_reset_cb;
static bool          m_dfu_started = false;


//////////////////////////////////////////////////////////////
//                   Timerdefinitionen                      //
//////////////////////////////////////////////////////////////
struct k_timer m_gdio_ind_ack_timer;
struct k_timer m_usdio_ind_ack_timer;
struct k_timer m_bond_timer;
struct k_timer m_alarm_restart_timer;
struct k_timer m_reset_prealarm_timer;
struct k_timer m_colorcode_input_timer;
struct k_timer m_colorcode_timeout_timer;
struct k_timer m_service_code_reset_timer;
struct k_timer m_reset_wrong_colorcode_input_timer;
struct k_timer m_alarm_timer;
struct k_timer m_check_alarm_timer;
struct k_timer m_six_min_timer;
struct k_timer m_plug_reactivation_timer;
struct k_timer m_bond_allowed_timer;
struct k_timer m_auth_timer;
struct k_timer m_gsm_send_timer;
//struct k_timer m_abort_fmna_user_pairing_timer;


static void timers_init()
{
    k_timer_init(&m_gdio_ind_ack_timer, gdio_ind_ack_timeout_handler, NULL);
    k_timer_init(&m_usdio_ind_ack_timer, usdio_ind_ack_timeout_handler, NULL);
    k_timer_init(&m_bond_timer, bond_timeout_handler, NULL);
    k_timer_init(&m_alarm_restart_timer, alarm_restart_timeout_handler, NULL);
    k_timer_init(&m_reset_prealarm_timer, reset_prealarm_timeout_handler, NULL);
    k_timer_init(&m_colorcode_input_timer, colorcode_input_timeout_handler, NULL);
    k_timer_init(&m_colorcode_timeout_timer, colorcode_timeout_handler, NULL);
    k_timer_init(&m_service_code_reset_timer, service_code_reset_timeout_handler, NULL);
    k_timer_init(&m_reset_wrong_colorcode_input_timer, reset_wrong_colorcode_input_timeout_handler, NULL);
    k_timer_init(&m_alarm_timer, alarm_timeout_handler, NULL);
    k_timer_init(&m_check_alarm_timer, check_alarm_timeout_handler, NULL);
    k_timer_init(&m_six_min_timer, six_min_timeout_handler, NULL);
    k_timer_init(&m_plug_reactivation_timer, plug_reactivation_timeout_handler, NULL);
    k_timer_init(&m_bond_allowed_timer, bond_allowed_timeout_handler, NULL);
    k_timer_init(&m_auth_timer, auth_timeout_handler, NULL);
    k_timer_init(&m_gsm_send_timer, gsm_send_timeout_handler, NULL);
    
//    k_timer_init(&m_abort_fmna_user_pairing_timer, abort_fmna_user_pairing_timeout_handler, NULL);
}


void work_bond_timeout_handler(struct k_work *work)
{
    LOG_DBG("work_bond_timeout_handler");
    m_bonding_mode_active = false;
    
    // Ein noch verbundenes Smartphone wird getrennt
    if(m_peer_started_bonding != BLE_CONN_HANDLE_INVALID)
    {
        m_disconnect_device |= m_peer_started_bonding;
    }
    
    // Im Werkszustand Advertising beenden
    if(m_factory_condition)
        advertising_stop();
    else
    {
        // Scan mit aktueller Konfiguration stoppen
        bt_scan_stop();
        
        if(m_num_fob_bonds > 0)
        {
            // Scan erneut mit anderer Konfiguration starten 
            scan_start();
        }
    }
}

K_WORK_DEFINE(work_bond_timeout, work_bond_timeout_handler);

void work_acc_sniff_handler(struct k_work* work)
{
    accelerometer_sniff();
}

K_WORK_DEFINE(work_acc_sniff, work_acc_sniff_handler);

void work_acc_sleep_handler(struct k_work* work)
{
    accelerometer_sleep();
}

K_WORK_DEFINE(work_acc_sleep, work_acc_sleep_handler);

void work_retention_write_handler(struct k_work* work)
{
    retention_write(m_ram, 0, &m_current_locking_state_chain, 1);
    retention_write(m_ram, 1, &m_current_locking_state, 1);
}

K_WORK_DEFINE(work_retention_write, work_retention_write_handler);


void work_fob_send_status_handler(struct k_work* work)
{
    bt_ili_client_send_mode(&m_ili_fob, m_fob_status);
}

K_WORK_DEFINE(work_fob_send_status, work_fob_send_status_handler);


K_WORK_DELAYABLE_DEFINE(work_start_advertising, start_adv_handler);

void start_adv_handler(struct k_work* work)
{
    if(m_advertising_is_active == false && m_peripheral_conn_count < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT)
    {
        LOG_DBG("advertising_start");
        
        int err;
#if ADV_TEST_LEGACY
        err = bt_le_adv_start(BLE_ADV_CONN_SLOW, m_adv_data, ARRAY_SIZE(m_adv_data), NULL, 0);
        LOG_DBG("bt_le_adv_start = %d", err);
#else
        struct bt_le_ext_adv_start_param ext_adv_start_param = {0};

        err = bt_le_ext_adv_start(m_ili_pro_adv_set, &ext_adv_start_param);
#endif
        if (err) {
            LOG_DBG("Advertising for ILI PRO set failed to start (err %d)", err);
            // Nach 5 Sekunden erneut versuchen das Advertisment zu starten
            k_work_schedule(&work_start_advertising, K_SECONDS(5));
        }
        else
        {
            LOG_DBG("ILI PRO advertising successfully started");
            m_advertising_is_active = true;
        }
    }
}


// static K_WORK_DELAYABLE_DEFINE(work_fmna_enable, fmna_enable_work_handle);

// static void fmna_enable_work_handle(struct k_work *item)
// {
//     int err;

// 	err = fmna_enable();
// 	if (err) {
// 		LOG_DBG("fmna_enable failed (err %d)", err);

// 		k_work_reschedule(&work_fmna_enable, K_SECONDS(1));
// 	} else {
// 		LOG_DBG("FMN enabled");
// 	}
// }


K_WORK_DELAYABLE_DEFINE(work_app_movement_timeout, app_movement_timeout_handler);

void app_movement_timeout_handler(struct k_work* work)
{
    LOG_DBG("app_movement_timeout_handler");
    // Keine Antwort vom Smartphone -> Abschließen
    set_motion_detection(true);
}

// static void fmna_sound_stopped_handler(struct k_work* item)
// {
//     LOG_DBG("fmna_sound_stopped_handler");

//     if(m_fmna_sound_started)
//     {
//         m_fmna_sound_started = false;
//         fmna_sound_completed_indicate();
//     }
// }

// static K_WORK_DELAYABLE_DEFINE(work_fmna_sound_stopped, fmna_sound_stopped_handler);

// static K_WORK_DELAYABLE_DEFINE(work_fmna_disable, fmna_disable_work_handle);

// static void fmna_disable_work_handle(struct k_work *item)
// {
//     int err;

// 	err = fmna_disable();
// 	if (err) {
// 		LOG_DBG("fmna_disable failed (err: %d)", err);

// 		k_work_reschedule(&work_fmna_disable, K_SECONDS(1));
// 	} else {
// 		LOG_DBG("FMN disabled");

// 		/* Reset the necessary flags. */
// 		m_fmna_location_available = false;
// 	}
// }

// static K_WORK_DEFINE(work_fmna_motion_detect, fmna_motion_detect_handle);

// static void fmna_motion_detect_handle(struct k_work *item)
// {
//     // Wurde manuell errechnet
//     // k = pow(2 * sin(10°/2), 2)
//     // k = 0,0304
//     // 1G beim Sensor entspricht dem Wert 65
//     // Somit ergibt sich der Threashold aus 65*65*0,0304 = 124
//     #define THRESHOLD_10_DEGREES    124

//     sample_t actual_sample = {0};

//     accelerometer_wakeup();
//     actual_sample = get_sample();

//     // Nach dem Auslesen des Samples wieder in den korrekten Modus wechseln
//     if(m_current_locking_state == STATUS_MOTOR_1_OPENED)
//         accelerometer_sleep();
//     else
//         accelerometer_sniff();

//     if(m_last_fmna_sample.x_msb != m_last_fmna_sample.y_msb != m_last_fmna_sample.z_msb != 0)
//     {
//         // Winkel abschätzen (mit Vorzeichen um +-5 Grad zu ermitteln)
//         int16_t dx = (int8_t)(m_last_fmna_sample.x_msb) - (int8_t)(actual_sample.x_msb);
//         int16_t dy = (int8_t)(m_last_fmna_sample.y_msb) - (int8_t)(actual_sample.y_msb);
//         int16_t dz = (int8_t)(m_last_fmna_sample.z_msb) - (int8_t)(actual_sample.z_msb);
//         int32_t dist_sq = (dx * dx) + (dy * dy) + (dz * dz);

//         //LOG_DBG("dx = %d\tdy = %d\tdz = %d", dx, dy, dz);
//         LOG_DBG("dist_sq = %d", dist_sq);

//         m_fmna_motion_detected = (dist_sq > THRESHOLD_10_DEGREES);
//     }

//     m_last_fmna_sample = actual_sample;

// 	if (m_fmna_motion_detected) {
// 		LOG_DBG("Motion detected in the last period");
// 	} else {
// 		LOG_DBG("No motion detected in the last period");
// 	}
// }


// K_WORK_DELAYABLE_DEFINE(work_start_fmna_pairing, start_fmna_pairing_handler);

// void start_fmna_pairing_handler(struct k_work* work)
// {
//     LOG_DBG("start_fmna_pairing_handler");
//     static uint8_t counter = 0;

//     if(m_fmna_user_pairing_active)
//     {
//         sample_t actual_sample = {0};

//         // Abbruchzähler erhöhen
//         counter++;

//         accelerometer_wakeup();
//         actual_sample = get_sample();
//         accelerometer_sleep();

//         LOG_DBG("x = %d\ty = %d\tz = %d\t", actual_sample.x_msb, actual_sample.y_msb, actual_sample.z_msb);
        
//         // NEO liegt auf dem Kopf
//         if(actual_sample.z_msb > 185 && actual_sample.z_msb < 195)
//         {
//             m_fmna_user_pairing_active = false;
//             counter = 0;
//             // Ton- oder LED-Ausgabe?
//             ili_piezo_play(ILI_PIEZO_SOUND_FMNA_START_PAIRING);

//             // Find-My Pairing starten
//             uint32_t err = fmna_pairing_mode_enter();
//             if (err) {
//                 LOG_DBG("Cannot enter the FMN pairing mode (err: %d)", err);
//             } else {
//                 LOG_DBG("%s the FMN pairing mode", m_fmna_pairing_mode_active ? "Extending" : "Enabling");
//                 m_fmna_pairing_mode_active = true;
//             }
//         }
//         else if(counter >= 6)
//         {
//             // Nach 30 Sek. wird die Prozedur abgebrochen
//             counter = 0;
//             m_fmna_user_pairing_active = false;
//             return;
//         }

//         // Work neu starten
//         k_work_schedule(&work_start_fmna_pairing, K_SECONDS(5));
//     }
// }


// Timeout-Handler zum Zurücksetzen des GDIO ACK
void gdio_ind_ack_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("gdio_ind_ack_timeout_handler");
	// Acknowledge kam nicht in vorgegebener Zeit
	m_gdio_indication_ack = true;
}


// Timeout-Handler zum Zurücksetzen des USDIO ACK
void usdio_ind_ack_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("usdio_ind_ack_timeout_handler");
	// Acknowledge kam nicht in vorgegebener Zeit
	m_usdio_indication_ack = true;
}

// Timeout-Handler zum Beenden des Bondingmodus
void bond_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("bond_timeout_handler");
    k_work_submit(&work_bond_timeout);
}


// Timeout-Handler zum Neustarten der Alarmprüfung
void alarm_restart_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("alarm_restart_timeout_handler");
    alarmcheck_start();
}

// Timeout-Handler zum Zurücksetzen des Flags, das das Anlernen alter Apps verhindert
void reset_incompatible_app_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("reset_incompatible_app_timeout_handler");
    m_reset_incompatible_app = false;
}

// Timeout-Handler zum Zurücksetzen des Voralarms
void reset_prealarm_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("reset_prealarm_timeout_handler");
    m_prealarm_fired = false;
}


/**@brief Timeout zur Eingabe des Farbcodes
 *
 */
void colorcode_input_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("----------colorcode_input_timeout_handler");

    // Gewählte Farbe festlegen
    m_colorcode_in[m_colorcode_in_index] = m_selected_color;

    m_colorcode_in_index++;
    m_selected_color = SELECTED_COLOR_INIT_VAL;
    led_off();

    // Timer für die Eingabe der nächsten Stelle im Farbcode starten
    k_timer_start(&m_colorcode_timeout_timer, TEN_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);

     // Auswertung der Eingabe
    if(m_colorcode_in_index >= 6)
    {
        LOG_DBG("Farbcode komplett");
        k_timer_stop(&m_colorcode_input_timer);
                    
        // Werkszustand per Farbcode
        if(m_service_code_state == SERVICE_CODE_ALLOWED)
        {
            LOG_DBG("Service-Farbcode erlaubt");
            // Timer zum Zurücksetzen des Flags anhalten
            k_timer_stop(&m_service_code_reset_timer);
            
            if(memcmp(m_uicr_data.reset_code, m_colorcode_in, 6) == 0)
            {
                LOG_DBG("Service-Farbcode korrekt");
                //Service-Farbcode eingegeben -> Werkszustand herstellen
                m_service_code_state = SERVICE_CODE_CORRECT;
                set_motion_detection(false);
            }
            // Eingegebener Code falsch
            else
            {
                LOG_DBG("Service-Farbcode falsch");
                // Erneute Eingabe des Service-Farbcodes verbieten
                m_service_code_state = SERVICE_CODE_INACTIVE;
                
                // rote LED blinken lassen   
                led_timed(LED_R, LED_ERROR);
                
                alarmcheck_start();
            }
        }
        else
        {
            LOG_DBG("Service-Farbcode nicht erlaubt");
            // Farbcode korrekt
            if(memcmp(m_colorcode, m_colorcode_in, 6) == 0 || memcmp(m_sharecode, m_colorcode_in, 6) == 0)
            {
                LOG_DBG("Normaler Farbcode korrekt");
                
                all_sounds_stop();
                set_motion_detection(false);
                
                // Falsche Eingaben des Farbcodes zurücksetzen
                m_wrong_colorcode_attempts = 0;
                k_timer_stop(&m_reset_wrong_colorcode_input_timer);
                
                if(memcmp(m_colorcode, m_colorcode_in, 6) == 0)
                {
                    // Neue Kopplung für eine gewisse Zeit erlauben, wenn Schloss geöffnet
                    m_bonding_allowed = true;
                    k_timer_start(&m_bond_allowed_timer, ONE_MIN_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
                }
            }
            else if(memcmp(m_colorcode_service_code_enable, m_colorcode_in, 6) == 0)
            {
                // Farbcode, der zum Starten der Eingabe des Service-Codes dient, eingegeben
                // Service-Code kann in den nächsten drei Minuten eingegeben werde
                m_service_code_state = SERVICE_CODE_ALLOWED;
                k_timer_start(&m_service_code_reset_timer, THREE_MIN_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
            }
            // Eingegebener Code falsch
            else
            {
                LOG_DBG("Normaler Farbcode falsch");
                // rote LED blinken lassen 
                led_timed(LED_R, LED_ERROR);
                               
                // Anzahl der Fehleingaben erhöhen
                m_wrong_colorcode_attempts++;
                
                // Wenn der Farbcode 5 mal falsch eingegeben wurde wird der Timer neugestartet
                // Damit wird die Eingabe für 3 Minuten gesperrt
                if(m_wrong_colorcode_attempts == 5)
                {
                    k_timer_stop(&m_reset_wrong_colorcode_input_timer);
                }
                
                // Timer zum Überwachen der Fehleingaben starten
                k_timer_start(&m_reset_wrong_colorcode_input_timer, RESET_WRONG_COLORCODE_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
                
                alarmcheck_start();
            }
        }
        
        abort_colorcode_input();
    }
}


/**@brief Timeout zur Überwachung der Eingabe aller sechs Farbcode-Stellen
 *
 */
void colorcode_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("colorcode_timeout_handler");
    // rote LED blinken lassen    
    led_timed(LED_R, LED_ERROR);
    abort_colorcode_input();
}



/** @brief Timeout zum setzen des Flags, das das Stoppen des Motors per Knopfdruck freigibt
*/
static void service_code_reset_timeout_handler(struct k_timer *timer)
{
    LOG_DBG("Eingabe Service-Farbcode verbieten");
    m_service_code_state = SERVICE_CODE_INACTIVE;
}


static void reset_wrong_colorcode_input_timeout_handler(struct k_timer* timer)
{
    LOG_DBG("reset_wrong_colorcode_input_timeout_handler");
    m_wrong_colorcode_attempts = 0;
}


static void alarm_timeout_handler(struct k_timer* timer)
{
    LOG_DBG("alarm_timeout_handler");

     if(m_signalsound_active)
        led_off();

    all_sounds_stop();
    // Wenn noch geschlossen, dann wird die Auswertung erneut gestartet
    alarmcheck_start();
}


static void check_alarm_timeout_handler(struct k_timer* timer)
{
    LOG_DBG("check_alarm_timeout_handler");

    LOG_DBG("m_check_alarm_counter = %d", m_alarmcounter);
    
    alarmcheck_start();
}


static void six_min_timeout_handler(struct k_timer* timer)
{
    m_batt_timeout_counter++;
    
    LOG_DBG("batt_timeout_handler() - %d", m_batt_timeout_counter);

    // Alle 24h messen, wenn nicht gerade geöffnet wird
    // Denn dann ist wahrscheinlich gerade eine Messung aktiv
    if(ili_motorcontroller_get_state() != MOTOR_OPEN && m_batt_timeout_counter >= 240)
    {
        LOG_DBG("Spannung messen");
        m_batt_timeout_counter = 0;
        ili_battery_start();
    }
}


static void plug_reactivation_timeout_handler(struct k_timer* timer)
{
    LOG_DBG("plug_reactivation_timeout_handler");
    m_chain_temp_disabled = false;
}


static void bond_allowed_timeout_handler(struct k_timer* timer)
{
    LOG_DBG("bond_allowed_timeout_handler");
    m_bonding_allowed = false;
}


static void auth_timeout_handler(struct k_timer* timer)
{
    LOG_DBG("auth_timeout_handler");

    // Für jeden Peer ein Zähler für das Timeout
    static uint8_t auth_timeout_counter[CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT];
    static uint8_t open_connections = 0;

    open_connections = 0;

    for(uint8_t i = 0; i < m_peripheral_conn_count; i++)
    {
        if(m_auth_state[i] != AUTHORISED)
        {
            LOG_DBG("Device %d is UNAUTHORISED with counter %d", i, auth_timeout_counter[i]);

            // Nach 10 Sek. wird die Verbindung getrennt
            if(auth_timeout_counter[i] >= 9)
            {
                // Index setzen, der in der Hauptschleife ausgewertet wird
                LOG_DBG("Disconnect Device %d", i);
                m_disconnect_device |= (1 << i);
            }
            else
            {
                auth_timeout_counter[i]++;
                open_connections++;
            }
        }
        else
        {
            LOG_DBG("Device %d is AUTHORISED", i);
        }
    }

    LOG_DBG("open_connections = %d", open_connections);
    // Wenn keine offenen Verbindungen mehr bestehen, kann der Timer beendet werden
    if(open_connections == 0)
    {
        // Timeout-Zähler zurücksetzen
        memset(auth_timeout_counter, 0, CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT);
        k_timer_stop(&m_auth_timer);
    }
}


void gsm_send_timeout_handler(struct k_timer* timer)
{
    LOG_DBG("gsm_send_timeout_handler - m_gsm_send_counter = %d", m_gsm_send_counter);
	
    // Anzahl der Elemente in der FIFO abfragen
    uint32_t count;
    count = k_msgq_num_used_get(&m_gsm_cmd_queue);
    
    // FIFO ist leer
    if (count == 0)
    {
        LOG_DBG("FIFO leer - Timer stoppen");
        // Wenn die FIFO leer ist, gibt es hier nichts mehr zu tun. 
        k_timer_stop(&m_gsm_send_timer);
        m_gsm_send_counter = 0;
    }
    else
    {
        // Befehl wurde bereits verschickt. ACK noch nicht erhalten -> Erneut versuchen
        if(m_gsm_send_counter < GPS_RETRY_COUNT)
        {
            // Befehl in Queue lassen
            m_gsm_send_cmd = true;
        }
        else
        {
            // Befehl aus Queue entfernen
            uint8_t command;
            k_msgq_get(&m_gsm_cmd_queue, &command, K_NO_WAIT); 
            
            m_gsm_send_counter = 0;
            
            // Es existiert mind. ein weiteres Element und der aktuelle Befehl
            if(count >= 2)
            {
                // Befehl versenden
                m_gsm_send_cmd = true;
            }
            else
            {
                // Keine Elemente mehr in der Queue -> Timer kann beendet werden
                k_timer_stop(&m_gsm_send_timer);
                
               if(m_gps_lpw_timeout == 0)
               {
                   // GPS deaktivieren
                   gps_off();
               }
            }
        }
    }
}


/*
    Timer zum Abbruch der Find-My Pairing Prozedur
*/
// static void abort_fmna_user_pairing_timeout_handler(struct k_timer* timer)
// {
//     LOG_DBG("abort_fmna_user_pairing_timeout_handler");

//     m_fmna_user_pairing_active = false;
//     // Beschleunigungssensor und Interrupt deaktivieren
//     k_work_submit(&work_acc_sleep);
//     gpio_remove_callback(pin_accel_int1.port, &accel_int1_cb_data);
// }


/**
 * @brief Funktion zur Umrechnung des Zweierkomplements
 * @param value - Zahl im Zweierkomplement
 * @return Absoluter Betrag
 */
static uint8_t comp8_t(uint8_t value)
{
	if (value > 0x7F)
	{
		value = ~value;
		value++;
		return value;
	}
	else
		return value;
}


/** @brief Gibt an, ob ein Nutzer verbunden und autorisiert ist
    @return true, wenn Nutzer gefunden sonst false
*/
static bool authorised_user_connected()
{
    for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
    {
        if(m_connected_peer[i].used && m_auth_state[i] == AUTHORISED)
            return true;
    }

    return false;
}


/**@brief // Bondingmodus starten
 */
static void new_bond()
{
    LOG_DBG("new_bond()");
    
    //Bonding erlauben
    m_bonding_mode_active = true;
    
    // LEDs ausschalten
    led_off();

    // Im Werkszustand wird nur nach Smartphones gesucht
    // Wenn bereits ein Smartphone angelernt ist, können auch Handsender verbunden werden
    if(m_factory_condition)
    {
        advertising_start();

        // Timer für Bonding-Dauer (neu-)starten
        k_timer_stop(&m_bond_timer);
        k_timer_start(&m_bond_timer, BOND_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);

        // Blaue LED blinkt, Bonding möglich
        led_timed(LED_B, LED_BONDING);
    }
    else
    {
        // Timer für Bonding-Dauer starten
        k_timer_start(&m_bond_timer, BOND_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);

        // Blaue LED blinkt, Bonding möglich
        led_timed(LED_B, LED_BONDING);

        // Wenn bereits ein Handsender verbunden ist, wird die Verbindung getrennt
        // Der Scan wird dann nach dem Disconnect gestartet
        if(m_bonded_fob_connected)
        {
            if(m_fob_conn != NULL)
            {
                bt_conn_disconnect(m_fob_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            }
        }
        else
        {
            // Scan mit aktueller Konfiguration stoppen
            if(m_num_fob_bonds > 0)
            {
                bt_scan_stop();
            }
            
            // Scan mit neuer Konfiguration starten
            scan_start();
        }
    }
}


/**@brief Zuordnung der Anzeigemuster zu den Einstellungen des LED-Treibers
 *
 * mode         - Muster, mit dem die LED angesteuert wird
 * brightness   - maximale Helligkeit in Prozent
 * period_ms    - Geschwindigkeit: Dauer eines kompletten Fade-Zyklus
 * duration_ms  - Dauer, die led_timed() die Anzeige laufen laesst (0 = endlos)
 */
typedef struct
{
    ili_led_mode_t  mode;
    uint8_t         brightness;
    uint16_t        period_ms;
    uint32_t        duration_ms;
} led_pattern_t;

static const led_pattern_t m_led_patterns[] =
{
    [LED_STATIC]     = { ILI_LED_MODE_STATIC,   50, 1000,  5000 },  // FIVE_SEC_TIMEOUT_INTERVAL
    [LED_ERROR]      = { ILI_LED_MODE_BLINK,    60,  200,  1000 },  // ONE_SEC_TIMEOUT_INTERVAL
    [LED_DISCONNECT] = { ILI_LED_MODE_BLINK,    60,  200,  1000 },  // ONE_SEC_TIMEOUT_INTERVAL
    [LED_BONDING]    = { ILI_LED_MODE_BREATHE,  40, 1500, 90000 },  // BOND_TIMEOUT_INTERVAL
    [LED_LOCKING]    = { ILI_LED_MODE_PULSE,    40,  800, 15000 },  // MOTOR_TIMEOUT_INTERVAL
    [LED_UNLOCKING]  = { ILI_LED_MODE_PULSE,    40,  800, 15000 },  // MOTOR_TIMEOUT_INTERVAL
    [LED_CHARGING]   = { ILI_LED_MODE_BREATHE,  40, 3000,     0 },  // laeuft bis zum Ende des Ladevorgangs
    [LED_ALARM]      = { ILI_LED_MODE_BLINK,    80,  250, 30000 },  // THIRTY_SEC_TIMEOUT_INTERVAL
    [LED_DOUBLE_TAP] = { ILI_LED_MODE_STATIC,   50, 1000,   500 },  // DOUBLE_TAP_TIMEOUT_INTERVAL
    [LED_SIGNAL]     = { ILI_LED_MODE_BLINK,    80,  400, 10000 },  // TEN_SEC_TIMEOUT_INTERVAL
};


/**@brief Anzeige beenden
 *
 * Waehrend des Ladevorgangs wird stattdessen die Ladeanzeige gestartet.
 */
void led_off()
{
    LOG_DBG("led_off");

    if(m_charge_active)
        led_faded(LED_R, LED_CHARGING);
    else
        ili_led_stop();
}


/**@brief LEDs dauerhaft aktivieren
 */
void led(uint8_t color)
{
    led_faded(color, LED_STATIC);
}


/**@brief Funktion zum Faden der LEDs
 *
 * Die Anzeige laeuft, bis sie mit led_off() beendet wird.
 */
void led_faded(uint8_t color, led_fading_t fading_type)
{
    LOG_DBG("led_faded");

    if((size_t)fading_type >= ARRAY_SIZE(m_led_patterns))
        return;

    const led_pattern_t* p_pattern = &m_led_patterns[fading_type];

    ili_led_config_t config = {
        .color       = color,
        .mode        = p_pattern->mode,
        .brightness  = p_pattern->brightness,
        .period_ms   = p_pattern->period_ms,
        .duration_ms = 0
    };

    ili_led_start(&config);
}


/**@brief Zeitlich begrenztes Aktivieren der LED
 *
 * Die Dauer ist im Anzeigemuster hinterlegt und wird vom LED-Treiber
 * ueberwacht.
 */
void led_timed(uint8_t color, led_fading_t fading_type)
{
    LOG_DBG("led_timed");

    if((size_t)fading_type >= ARRAY_SIZE(m_led_patterns))
        return;

    const led_pattern_t* p_pattern = &m_led_patterns[fading_type];

    ili_led_config_t config = {
        .color       = color,
        .mode        = p_pattern->mode,
        .brightness  = p_pattern->brightness,
        .period_ms   = p_pattern->period_ms,
        .duration_ms = p_pattern->duration_ms
    };

    ili_led_start(&config);
}


/**
 * @brief Starten des Alarmtons
 */
static void alarmsound_start()
{
    if(m_alarmsound_active == false)
    {
        LOG_DBG("alarmsound_start");
        
        if(m_settings.alarm.prealarm)
        {
            LOG_DBG("Stop m_reset_prealarm_timer");
            k_timer_stop(&m_reset_prealarm_timer);
        }
        
    //    PRINTLN("play_alarmsound - top_value = %d", m_top_value);  
        
        // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fpwm.html&cp=2_2_0_46_0&anchor=concept_l3d_smw_nr
        // https://devzone.nordicsemi.com/f/nordic-q-a/14954/custom-pwm-base-clock-frequency
        // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.2.0%2Fhardware_driver_pwm.html&cp=4_0_0_2_0_8
        // https://devzone.nordicsemi.com/f/nordic-q-a/40950/how-can-i-create-1-7mhz-2-5mhz-3mhz-duty-cycle-50-by-pwm-in-nrf52832
        // https://devzone.nordicsemi.com/f/nordic-q-a/15018/getting-started-with-pwm-on-nrf52
        
        // Beispiel für GPIO-Toggle mit PPI und Timer (damit könnte man vielleicht die PWM ersetzen, die die 250 kHz erzeugt)
        // https://github.com/bjornspockeli/nRF52_ppi_timper_gpiote_example/blob/master/Readme.md
        // https://embeddedexplorer.com/nrf52-ppi-tutorial/
        
        
        m_alarmsound_active = true;
        
        // Alarm aktiv an Smartphone übertragen	
        send_status(BLE_CONN_HANDLE_ALL, STATUS_ALARM_ON);
        // Alarm an Handsender übertragen
        m_fob_status = ILI_C_ALARM_ACTIVE;
        k_work_submit(&work_fob_send_status);
        
        // Wenn kein stiller Alarm aktiv, Piezo initialisieren
        if(m_settings.alarm.silent == false && m_dnd_mode_active == false)
        {
            // Piezo aktivieren
            ili_piezo_play(ILI_PIEZO_SOUND_ALARM);

            // Rote LED pulsieren lassen während des Alarms
            led_timed(LED_R, LED_ALARM);
            // Test mit top_value von 50 - 80
            // Entspricht einem Frequenzbereich von 5,000 kHz bis 3,125 kHz (Schrittweite 62,5 Hz)
        }
        
        // Timer zum Beenden des Alarms starten
        k_timer_start(&m_alarm_timer, THIRTY_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
    }
}


/**
 * @brief Starten des Voralarms
 */
static void prealarm_start()
{
    if(m_prealarm_active == false)
    {
        LOG_DBG("prealarm_start");
        
        m_prealarm_active = true;
        m_prealarm_fired = true;
        
        // Piezo aktivieren
        ili_piezo_play(ILI_PIEZO_SOUND_PREALARM);

        // Timer zum Beenden des Alarms starten
        k_timer_start(&m_alarm_timer, PREALARM_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
    }
}


/**
 * @brief Starten des Signaltons
 */
static void signalsound_start()
{    
    if(m_signalsound_active == false)
    {
        LOG_DBG("signalsound_start");
        
        m_signalsound_active = true;
        
        // Piezo aktivieren
        ili_piezo_play(ILI_PIEZO_SOUND_SIGNAL);

        // LEDs aktivieren
        led_timed(LED_R, LED_SIGNAL);

        // Timer zum Beenden des Alarms starten
        k_timer_start(&m_alarm_timer, TEN_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
    }
}


/**
 * @brief Starten eines Warn-Tons
 */
void beep_start(ili_piezo_mode_t beep_type)
{
    LOG_DBG("beep_start - %d", beep_type);
    LOG_DBG("m_settings.sound.ble = %d", m_settings.sound.ble);
    LOG_DBG("m_settings.sound.mode = %d", m_settings.sound.mode);

    // Töne nur abspielen, wenn DND Modus nicht aktiv
    if(m_dnd_mode_active == false || beep_type == ILI_PIEZO_SOUND_FMNA_OWNER || beep_type == ILI_PIEZO_SOUND_FMNA_NON_OWNER)
    {
        if(beep_type == ILI_PIEZO_SOUND_DISARMED && (m_settings.sound.mode & SOUND_CONF_OPEN))
        {
            ili_piezo_play(beep_type);
        }
        else if((beep_type == ILI_PIEZO_SOUND_ARMED && (m_settings.sound.mode & SOUND_CONF_CLOSE))
                || beep_type == ILI_PIEZO_SOUND_AUTO_CLOSE)
        {
            ili_piezo_play(beep_type);
        }
        else if((beep_type == ILI_PIEZO_SOUND_WARNING || beep_type == ILI_PIEZO_SOUND_LOW_BATT) && (m_settings.sound.mode & SOUND_CONF_WARNING))
        {
            ili_piezo_play(beep_type);
        }
        else if(beep_type == ILI_PIEZO_SOUND_FMNA_OWNER || beep_type == ILI_PIEZO_SOUND_FMNA_NON_OWNER)
        {
            ili_piezo_play(beep_type);
        }
    }
}


/**
 * @brief Die individuell festgelegten Farbcodes dekodieren
 */
static void set_colorcodes()
{
    LOG_RAW("set_colorcodes - ");
	for (int i = 0; i < 3; i++)
	{
		m_colorcode[i * 2]     = m_settings.colorcode_flash[i] / 10;
		m_colorcode[i * 2 + 1] = m_settings.colorcode_flash[i] % 10;
        
        m_sharecode[i * 2]     = m_settings.sharecode_flash[i] / 10;
		m_sharecode[i * 2 + 1] = m_settings.sharecode_flash[i] % 10;
        LOG_RAW("%d %d ", m_colorcode[i * 2], m_colorcode[i * 2 + 1]);
	}	
    LOG_RAW("\r\n");
}


/**
 * @brief Liefert die Anzahl der gekoppelten Smartphones
 */
uint8_t get_bond_count()
{
	uint8_t num_peers = 0;
	
	// Benutzte Peer-Daten ausgeben
	for(uint8_t i = 0; i < MAX_PEER_COUNT; i++)
	{
		if(m_peer_data[i].is_used)
		{	
			num_peers++;
		}
	}
	
	return num_peers;
}


/**
 * @brief Hinzufügen der Peer-ID und HW-ID an eine freie Stelle im Speicher
 */
static uint8_t add_peer_data(uint8_t* new_hw_id)
{
	LOG_DBG("add_peer_data()");
	
	uint8_t peer_index;
	
	// Prüfen, ob HW-ID bereits vorhanden ist
	for(peer_index = 0; peer_index < MAX_PEER_COUNT; peer_index++)
	{
		if(memcmp(m_peer_data[peer_index].hw_id, new_hw_id, HW_ID_LEN) == 0)
		{
			LOG_DBG("Vorhandenen Peer gefunden");
			break;
		}
        // TODO: Schleifen können zusammengelegt werden.
        // Den nächsten freien Platz kann man auch hier schon ermitteln
	}
	
	// HW-ID nicht gefunden
	if(peer_index == MAX_PEER_COUNT)
	{
		// Nächsten freien Platz in der Liste finden
		for(peer_index = 0; peer_index < MAX_PEER_COUNT; peer_index++)
		{
			if(m_peer_data[peer_index].is_used != true)
			{
				break;
			}
		}
	}
	LOG_DBG("Peer Index: %d", peer_index);
	
	// kein freier Platz mehr in der Liste
	if(peer_index == MAX_PEER_COUNT)
	{
		return ENOMEM;
	}
	
	memcpy(m_peer_data[peer_index].hw_id, new_hw_id, HW_ID_LEN);
	m_peer_data[peer_index].auth_id = peer_index;
	
	// Aktuell verbundenes Gerät festlegen
    if(m_peer_started_bonding != BLE_CONN_HANDLE_INVALID)
        m_connected_peer[m_peer_started_bonding].peer_data = &m_peer_data[peer_index];
        
    return APP_SUCCESS;
}


/**
 * @brief Löschen der Peer-Daten einer bestimmten ID
 * @param auth_id ID des zu löschenden Peers
 */
uint8_t delete_peer_data(uint16_t auth_id)
{
	LOG_DBG("delete_peer_data()");
	
	// Feststellen, ob der zu löschende Peer existiert
	if(m_peer_data[auth_id].is_used == false)
	{
		// Peer-ID nicht gefunden
		send_status(BLE_CONN_HANDLE_ALL, STATUS_BOND_NOT_FOUND);
		return -ENXIO;
	}

    LOG_HEXDUMP_DBG(m_peer_data[auth_id].hw_id, HW_ID_LEN, "Datensatz gelöscht");
    
	// Peer-Daten löschen
	m_peer_data[auth_id].auth_id = auth_id;
	memset(m_peer_data[auth_id].hw_id, 0xFF, HW_ID_LEN);
	m_peer_data[auth_id].is_used = false;

	// Peer-Daten speichern
	m_update_peer_auth_id = auth_id;
	
	return APP_SUCCESS;
}


static peer_context_t* get_free_peer_context(void)
{
	for(int i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++) {
		if(!m_connected_peer[i].used) {
            LOG_DBG("get_free_peer %d", i);
			m_connected_peer[i].used = true;
            m_connected_peer[i].index = i;
			return &m_connected_peer[i];
		}
	}
	return NULL;
}


static peer_context_t* get_peer_context_from_conn(struct bt_conn *conn)
{
	for(int i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++) {
		if(m_connected_peer[i].used && m_connected_peer[i].conn == conn) {
			return &m_connected_peer[i];
		}
	}
	return NULL;
}


/** @brief Gibt an, ob ein Nutzer autorisiert ist und sich in der Nähe befindet
    @return true, wenn Nutzer gefunden sonst false
*/
static bool authorised_user_is_nearby()
{
    for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
    {
        if(m_auth_state[i] == AUTHORISED && m_rssi_user_state[i] == RSSI_STATE_NEAR)
        {
            return true;
        }
    }

    return false;
}


static void ble_services_init()
{
    // DB-Discovery Modul initialisieren
 /*   db_discovery_init();
    
    uint32_t           err_code;
    ble_ili_init_t     ili_init;
    ble_dfu_buttonless_init_t dfus_init = {0};
    
    // Initialisieren des I LOCK IT Services für die App
    memset(&ili_init, 0, sizeof(ili_init));

    ili_init.data_handler = ili_data_handler;

    err_code = ble_ili_init(&m_ili, &ili_init);
    APP_ERROR_CHECK(err_code);
    
    // Initialisieren des I LOCK IT Client Services des Handsenders
    ble_ili_c_init_t ili_c_init;

    ili_c_init.evt_handler   = ble_ili_c_evt_handler;
    ili_c_init.error_handler = ili_c_error_handler;
    ili_c_init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_ili_c_init(&m_ili_c, &ili_c_init);
    APP_ERROR_CHECK(err_code);
    
    // Initialisieren des Buttonless-DFU Services
    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
    
    // Initialisierung für Verbindungszustände
    ble_conn_state_init();
    */

    ili_service_init(&ili_callbacks);

	struct bt_ili_client_init_param init = {
		.cb = {
			.received = ble_mode_data_received,
			.sent = ble_mode_data_sent,
            .unsubscribed = ble_mode_unsubscribed,
		}
	};

    bt_ili_client_init(&m_ili_fob, &init);

    // BLE-Zustände initialisieren
    for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
        m_auth_state[i] = UNAUTHORISED;

    // Device-Name setzen
#if ADV_TEST_LEGACY
    // Fester Testname, unabhaengig vom UICR-Inhalt
    int name_err = bt_set_name(ADV_TEST_NAME);
    LOG_DBG("bt_set_name(\"%s\") = %d", ADV_TEST_NAME, name_err);
#else
    bt_set_name(m_uicr_data.advertising_name);
#endif
}


static void advertising_init()
{
#if ADV_TEST_LEGACY
    // Beim Legacy-Advertising werden Parameter und Daten erst in
    // bt_le_adv_start() uebergeben -> hier ist nichts zu initialisieren.
    LOG_DBG("advertising_init: Legacy-Advertising, Name = \"%s\" (%d Byte Payload)",
            ADV_TEST_NAME, 3 + 2 + (int)(sizeof(ADV_TEST_NAME) - 1));
#else
    int err;
    struct bt_le_adv_param param = {0};

    param.id           = ILI_PRO_BT_ID;
    param.sid          = ILI_PRO_BT_ID;
    param.options      = (BT_LE_ADV_OPT_CONN);
    param.interval_min = APP_ADV_INTERVAL_MIN;
    param.interval_max = APP_ADV_INTERVAL_MAX;
    err = bt_le_ext_adv_create(&param, NULL, &m_ili_pro_adv_set);
    if (err) {
        LOG_DBG("Could not create ILI PRO advertising set (err %d)", err);
    }

    const struct bt_data adv_data[] = {
	    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))
    };

    err = bt_le_ext_adv_set_data(m_ili_pro_adv_set, adv_data, ARRAY_SIZE(adv_data),
	     NULL, 0);
    if (err) {
        LOG_DBG("Could not set data for ILI NEO advertising set (err %d)", err);
    }

    LOG_DBG("advertising_init erfolgreich");
#endif
}


static void advertising_start()
{
    k_work_schedule(&work_start_advertising, K_SECONDS(1));
}


static void advertising_stop()
{
    if(m_advertising_is_active)
    {
        LOG_DBG("advertising_stop");
        
#if ADV_TEST_LEGACY
        int err = bt_le_adv_stop();
        LOG_DBG("bt_le_adv_stop = %d", err);
#else
        int err = bt_le_ext_adv_stop(m_ili_pro_adv_set);
        LOG_DBG("bt_le_ext_adv_stop = %d", err);
#endif
        
        if(err == 0)
            m_advertising_is_active = false;
        
        TD("Neustart, falls fehlerhaft?");
    }
}


static bool adv_data_found(struct bt_data *data, void *user_data)
{
	switch (data->type) {
	case BT_DATA_NAME_COMPLETE:
		/* Check the name filter. */
		if(strncmp(m_keyfob_name, data->data, strlen(m_keyfob_name)) == 0)
        {
            LOG_DBG("KEYFOB gefunden");
            bt_addr_le_t* addr = (bt_addr_le_t*)user_data;

	        // Scan stoppen und Verbindung aufbauen
	        bt_scan_stop();

            int err = bt_conn_le_create(addr,
			       BT_CONN_LE_CREATE_CONN,
			       BT_LE_CONN_PARAM_DEFAULT, &m_fob_conn);

	        LOG_DBG("Connecting (%d)", err);
 
            if(err)
            {
                scan_start();
            }
        }
		break;

	default:
		break;
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
       struct net_buf_simple *ad)
{
    bt_data_parse(ad, adv_data_found, (void*)addr);
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", (addr));
}


static void pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	bt_conn_auth_pairing_confirm(conn);

	LOG_INF("Pairing confirmed: %s", (addr));
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

    static struct bt_conn_info conn_info;
    bt_conn_get_info(conn, &conn_info);

	if(conn_info.id != ILI_PRO_BT_ID)
        return;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Acceptlist neu erstellen
    if(bonded)
    {
        // Advertisment stoppen, da sonst die Acceptlist nicht geändert werden kann
        advertising_stop();

        // Filter-List leeren
        int err = bt_le_filter_accept_list_clear();
        if (err) {
            LOG_INF("Cannot clear Filter Accept List (err: %d)\n", err);
            return;
        }

        int bond_cnt = 0;
        bt_foreach_bond(ILI_PRO_BT_ID, setup_accept_list_cb, &bond_cnt);
        
        // Nach dem Herstellen einer neuen Verbindung kann der Bonding-Modus beendet werden
        m_num_fob_bonds = bond_cnt;
        k_timer_stop(&m_bond_timer);
        m_bonding_mode_active = false;
    
        LOG_DBG("Peers added to filterlist = %d", bond_cnt);

        // Advertisment neu starten
        advertising_start();
    }

	LOG_DBG("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_DBG("Pairing failed conn: %s, reason %d", addr, reason);
}


// Callback für bt_foreach Funktion
static void setup_accept_list_cb(const struct bt_bond_info *info, void *user_data)
{
	int *bond_cnt = user_data;

	if ((*bond_cnt) < 0) {
		return;
	}

	int err = bt_le_filter_accept_list_add(&info->addr);
	LOG_INF("Added following peer to accept list: %x %x %x %x %x %x\n", info->addr.a.val[0],
		info->addr.a.val[1], info->addr.a.val[2], info->addr.a.val[3], info->addr.a.val[4], info->addr.a.val[5]);
	if (err) {
		LOG_INF("Cannot add peer to filter accept list (err: %d)\n", err);
		(*bond_cnt) = -EIO;
	} else {
		(*bond_cnt)++;
	}
}


static void scan_init(void)
{
	int err;

	bt_scan_init(&SCAN_INIT_PARAMS);

    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		LOG_DBG("Failed to register authorization info callbacks.");
		return;
	}

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return;
	}

    // Filter-List initialisieren
    err = bt_le_filter_accept_list_clear();
	if (err) {
		LOG_INF("Cannot clear Filter Accept List (err: %d)\n", err);
		return;
	}

	int bond_cnt = 0;
	bt_foreach_bond(ILI_PRO_BT_ID, setup_accept_list_cb, &bond_cnt);
	
    LOG_DBG("Peers added to filterlist = %d", bond_cnt);

    // Anzahl gekoppelter Handsender setzen
    m_num_fob_bonds = bond_cnt;
}


static int scan_start(void)
{
    LOG_DBG("scan_start - m_bonding_mode_active = %d", m_bonding_mode_active);
    int err;
    if(m_bonding_mode_active)
    {
        LOG_DBG("scan start BONDING");
        err = bt_le_scan_start(&SCAN_PARAM_BONDING, device_found);
    }
    else
    {
        LOG_DBG("scan start WHITELIST");
        err = bt_le_scan_start(&SCAN_PARAM_WHITELIST, device_found);
    }
    
	if (err) {
		LOG_DBG("Scanning failed to start (err %d)\n", err);
	}

	return err;
}


static void gatt_discover(struct bt_conn *conn)
{
	int err;

	if (conn != m_fob_conn) {
		return;
	}

	err = bt_gatt_dm_start(conn, BT_UUID_ILI_C_SERVICE, &discovery_cb, NULL);
	if (err) {
		LOG_DBG("Could not start the discovery procedure, error code: %d", err);
	}
}


static void ble_mode_unsubscribed(struct bt_ili_client *ili)
{
    LOG_DBG("ble_mode_unsubscribed");
    bt_ili_subscribe_mode(&m_ili_fob);
}

static void ble_mode_data_received(struct bt_ili_client *ili, const uint8_t *const data, uint16_t len)
{
    if(len == 0)
    {
        return;
    }

    LOG_DBG("ble_mode_data_received");
    LOG_RAW("data: ");
    for(uint8_t i = 0; i < len;i++)
    {
        LOG_RAW("%.2X", data[i]);
    }
    LOG_RAW("\n");

    switch(data[0])
    {
        case ILI_C_LOCK_ACTION:
        {
            LOG_DBG("ILI_C_LOCK_ACTION");
            if(ili_motorcontroller_get_state() == MOTOR_STOP)
            {
                set_motion_detection(m_current_locking_state == STATUS_MOTOR_1_OPENED);
            }
        }
        break;

        case ILI_C_DEACTIVATE_ALARM:
        {
            LOG_DBG("ILI_C_DEACTIVATE_ALARM");
            all_sounds_stop();
            alarmcheck_stop();
            // Einige Sekunden warten, bis der Alarm wieder aktiviert wird
            k_timer_start(&m_alarm_restart_timer, FIVE_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
        }
        break;

        case ILI_C_START_BONDING_MODE:
        {
            LOG_DBG("ILI_C_START_BONDING_MODE");

            // Wenn noch Platz in der Bondingliste ist, wird ein neues Bonding gestartet
            if(m_num_fob_bonds < MAX_FOB_BONDS || m_num_app_bonds < MAX_PEER_COUNT)
            {
                send_status(BLE_CONN_HANDLE_ALL, STATUS_BOND_FROM_HS);
                new_bond();
            }
            else
            {
                // Notification, dass die max. Anzahl an Geräten gebondet ist
                send_status(BLE_CONN_HANDLE_ALL, STATUS_MAX_BONDS);
                // rote LED blinken lassen  
                led_timed(LED_R, LED_ERROR);
            }
        }
        break;
    }
}


static void ble_mode_data_sent(struct bt_ili_client *ili,uint8_t err, const uint8_t *const data, uint16_t len)
{
    LOG_DBG("ble_mode_data_sent");
}


void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    //uint32_t connection_interval = interval;         // in ms
    //uint16_t supervision_timeout = timeout*10;          // in ms
    //LOG_INF("Connection parameters updated: interval %d ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
}


static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	static struct bt_conn_info conn_info;

	LOG_INF("Connected\n");

	bt_conn_get_info(conn, &conn_info);

	if(conn_info.id != ILI_PRO_BT_ID)
        return;

	if (conn_info.role == BT_CONN_ROLE_PERIPHERAL)
	{
        int err = 0;
        
        // Wenn noch keine Verbindung besteht, wird Find-My deaktiviert
        // if(m_peripheral_conn_count == 0)
        // {
        //     /* FMNA specification guidelines for pair before use accessories:
        //     * Cancel the FMN pairing mode once the Bluetooth peer connects to the
        //     * device for its primary purpose (HR sensor in the context of this
        //     * sample).
        //     */
        //     if (m_fmna_pairing_mode_active) {
        //         err = fmna_pairing_mode_cancel();
        //         if (err) {
        //             LOG_DBG("Cannot cancel the FMN pairing mode (err: %d)", err);
        //         } else {
        //             LOG_DBG("FMN pairing mode cancelled");

        //             m_fmna_pairing_mode_active = false;
        //         }
        //     }

        //     /* FMNA specification guidelines for pair before use accessories:
        //     * Disable the FMN paired advertising once the Bluetooth peer connects
        //     * to the device for its primary purpose (HR sensor in the context of
        //     * this sample).
        //     */
        //     err = fmna_paired_adv_disable();
        //     if (err) {
        //         LOG_DBG("fmna_paired_adv_disable failed (err %d)", err);
        //         return;
        //     }
        // }

        // Scan neustarten, da sonst keine Verbindung mit einem Handsender möglich ist
        if(m_factory_condition == false && (m_num_fob_bonds > 0 || m_bonding_mode_active) && m_fob_conn == NULL)
        {
            bt_scan_stop();
            scan_start();
        }

        if (conn_err) {
            LOG_ERR("Connection failed (err %u)\n", conn_err);
            return;
	    }

        m_advertising_is_active = false;

        m_peripheral_conn_count++;

        peer_context_t* peripheral = get_free_peer_context();
		if (peripheral) {
			peripheral->conn = conn;
            LOG_DBG("conn setzen state: %d", conn_info.state);
		}

        // Timer zur Überwachung der Verbindung starten
        //k_timer_start(&m_auth_timer, ONE_SECOND_TIMEOUT_INTERVAL, ONE_SECOND_TIMEOUT_INTERVAL);

		if(m_peripheral_conn_count < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT)
		{
			advertising_start();
		}
	}
    else if(conn_info.role == BT_CONN_ROLE_CENTRAL)
    {
        LOG_DBG("Handsender verbunden!");
        
        if (conn_err) {
            LOG_ERR("Connection failed (err %u)\n", conn_err);
            if(conn == m_fob_conn)
            {
                bt_conn_unref(m_fob_conn);
                m_fob_conn = NULL;
                scan_start();
            }
            return;
	    }

        // Bonding-Prozess starten, falls noch kein Bonding besteht
        // Wenn ein Fehler geworfen wird, besteht bereits ein Bonding
        int err = bt_conn_set_security(conn, BT_SECURITY_L2);
        LOG_DBG("bt_conn_set_security = %d", err);
    }
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);
	static struct bt_conn_info conn_info;

	bt_conn_get_info(conn, &conn_info);

	if(conn_info.id != ILI_PRO_BT_ID)
        return;

	if (conn_info.role == BT_CONN_ROLE_PERIPHERAL) 
	{
		peer_context_t* peripheral = get_peer_context_from_conn(conn);

        if(peripheral != NULL)
        {
            if(m_auth_state[peripheral->index] != UNAUTHORISED && m_bonding_mode_active == false)
            {
                led_timed(LED_B, LED_DISCONNECT);
            }

            peripheral->peer_data = NULL;
            peripheral->used = false;
            memset(peripheral->ltk, 0, LTK_LEN);

            //bt_conn_unref(peripheral->conn);
            
            ili_reset_parser(peripheral->index);

            // Empfangsflags zurücksetzen
            m_usdio_data_received &= (0 << peripheral->index);
            m_gdio_data_received  &= (0 << peripheral->index);

            // Wenn der Peer, der sich gerade trennt das Bonding begonnen hatte, muss das Flag zurückgesetzt werden
            if(m_peer_started_bonding == peripheral->index)
                m_peer_started_bonding = BLE_CONN_HANDLE_INVALID;
                
            // Peer-Daten ungültig machen
            m_auth_state[peripheral->index] = UNAUTHORISED;
                
            // Schließzähler zurücksetzen
            m_lock_counter[peripheral->index] = 0;
                
            // RSSI-Zustand zurücksetzen
            m_rssi_user_state[peripheral->index] = RSSI_STATE_FAR;
            
        }

        m_peripheral_conn_count--;

        // Wenn keine Verbindung mehr besteht, wird Find-My wieder akiviert
        // if(m_peripheral_conn_count == 0)
        // {
        //     /* FMNA specification guidelines for pair before use accessories:
        //     * Enable the FMN paired advertising once the Bluetooth connection
        //     * for the accessory primary purpose (HR sensor in the context of
        //     * this sample) is terminated.
        //     */
        //     fmna_paired_adv_enable();
        // }

        // Wenn das Advertising gestoppt wurde, weil die max. Anzahl erreicht wurde, 
        // wird es hier beim Disconnect wieder gestartet.
        if(m_bootloader_prepare == false && m_peripheral_conn_count == CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT - 1)
        {
            // Advertising is not running when all connections are taken, and must therefore be started.
           advertising_start();
        }


        if(m_factory_condition && m_bonding_mode_active == false)
        {
            // Alle Daten löschen und in Werkszustand zurückkehren
            m_start_factory_reset = true;
        }

        LOG_DBG("BLE_GAP_EVT_DISCONNECTED reset peer data");

		if (peripheral) {
			peripheral->used = false;
		}

        // Wenn keine Geräte mehr verbunden sind, kann die RSSI-Auswertung gestoppt werden
        if(m_peripheral_conn_count == 0)
            m_stop_rssi = true;
	}
    else if(conn_info.role == BT_CONN_ROLE_CENTRAL)
    {
        if (m_fob_conn != conn) {
		    return;
	    }

        char addr[BT_ADDR_LE_STR_LEN];

	    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

        bt_conn_unref(m_fob_conn);
        m_fob_conn = NULL;

        LOG_DBG("CENTRAL: Disconnected: %s (reason %u)", addr, reason);
            
        if(m_bonded_fob_connected)
        {
            // Blau blinken
            if(m_bonding_mode_active == false)
            {
                led_timed(LED_B, LED_DISCONNECT);
            }
        }
            
        m_bonded_fob_connected = false;
            
        if((m_num_fob_bonds > 0 || m_bonding_mode_active) && m_bootloader_prepare == false)
            scan_start();
    }
}


static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err bt_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    static struct bt_conn_info conn_info;
    bt_conn_get_info(conn, &conn_info);

	if(conn_info.id != ILI_PRO_BT_ID)
        return;

	if (!bt_err) {
		LOG_DBG("Security changed: %s level %u", addr, level);
	} else {
		LOG_DBG("Security failed: %s level %u err %d", addr, level,
			bt_err);

        // Bonding löschen, falls es zu dem Handsender bereits ein Bonding gibt
        // und der Handsender aber im Werkszustand ist.
        bt_unpair(ILI_PRO_BT_ID, conn_info.le.dst);
        bt_conn_disconnect(m_fob_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
	}

    // Verbunden zu App -> Bonding löschen
    if(m_bonding_mode_active && conn_info.role == BT_CONN_ROLE_PERIPHERAL)
    {
        bt_unpair(ILI_PRO_BT_ID, conn_info.le.dst);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    m_bonded_fob_connected = true;
    // Blaue LED anschalten bei Verbindungsaufbau              
    led_timed(LED_B, LED_STATIC);

    if(m_bonding_mode_active == false)
    {
        LOG_DBG("Gebondeter Handsender hat sich verbunden");
        // Wenn der Alarm beim Verbinden aktiv ist wird der Alarm beendet
        if(m_alarmsound_active)
        {
            all_sounds_stop();
            alarmcheck_stop();
        }
                    
        // Bei Verbindungsaufbau Schließbefehl ausführen
        set_motion_detection(m_current_locking_state == STATUS_MOTOR_1_OPENED);
    }

	gatt_discover(conn);
}


static void discovery_completed_cb(struct bt_gatt_dm *dm,
				   void *context)
{
	int err;

	LOG_DBG("The discovery procedure succeeded");

	bt_gatt_dm_data_print(dm);

	err = bt_ili_handles_assign(dm, &m_ili_fob);
	if (err) {
		LOG_DBG("Could not init BAS client object, error: %d", err);
	}

    err = bt_ili_subscribe_mode(&m_ili_fob);
	if (err) {
		LOG_DBG("Cannot subscribe to ILI mode value notification "
			"(err: %d)", err);
		/* Continue anyway */
	}
	
	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_DBG("Could not release the discovery data, error "
		       "code: %d", err);
	}
}


static void discovery_service_not_found_cb(struct bt_conn *conn,
					   void *context)
{
	LOG_DBG("The service could not be found during the discovery");
}


static void discovery_error_found_cb(struct bt_conn *conn,
				     int err,
				     void *context)
{
	LOG_DBG("The discovery procedure failed with %d", err);
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
    .security_changed = security_changed,
    .le_param_updated = on_le_param_updated,
};


void rssi_init()
{
    m_rssi_thread_id = k_thread_create(&m_rssi_thread, m_rssi_stack, RSSI_STACKSIZE,
				      get_rssi, NULL, NULL, NULL,
				      K_LOWEST_APPLICATION_THREAD_PRIO, K_USER, K_NO_WAIT);
    
    k_thread_suspend(m_rssi_thread_id);
}


/** @brief Starten der RSSI-Auswertung zum automatisches Öffnen für ein Geräte
*/
void rssi_start()
{
    if(m_rssi_active == false)
    {
        // Alle RSSI-Werte zurücksetzen
        memset(m_rssi_values, 100, sizeof(m_rssi_values));
        memset(m_rssi_pos, 0, CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT);
        for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
        {
            m_rssi_sum[i] = 100 * RSSI_AVG_COUNT; 
        }
        memset(m_rssi_user_state, RSSI_STATE_FAR, CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT);

        // Thread für die RSSI-Abfrage fortsetzen
        k_thread_resume(m_rssi_thread_id);
        m_rssi_active = true;
    }
}


/** @brief Stoppen der RSSI-Auswertung für automatisches Öffnen
*/
void rssi_stop()
{
    m_rssi_active = false;

    // Anhalten des Threads
    // Dabei wird auf die Freigabe des Mutex gewartet
    k_mutex_lock(&m_rssi_mutex, K_FOREVER);
    k_thread_suspend(m_rssi_thread_id);
    k_mutex_unlock(&m_rssi_mutex);
}


uint16_t movingAvg(uint8_t *p_arrNumbers, uint16_t *p_sum, uint8_t pos, uint8_t len, uint8_t nextNum)
{
    //LOG_DBG("p_sum = %d", *p_sum);
    //Subtract the oldest number from the prev sum, add the new number
    *p_sum = *p_sum - p_arrNumbers[pos] + nextNum;
    
    //LOG_DBG("%d - %d + %d = %d", *p_sum, p_arrNumbers[pos], nextNum, *p_sum);
    //Assign the nextNum to the position in the array
    p_arrNumbers[pos] = nextNum;
    //return the average
    return *p_sum / len;
}


const char *rssi_str(uint8_t state)
{
    /* Array to map FDS return values to strings. */
    static char const * rssi_str[] =
    {
        "RSSI_STATE_FAR",
        "RSSI_STATE_NEAR"
    };

    return rssi_str[state];
}


/**@brief // Auswertung der Empfangenen RSSI Werte und entsprechende Reaktion
   @param rssi_akt Aktueller RSSI-Wert
 */
static void rssi_check(uint16_t conn_handle, uint8_t rssi_new)
{    
//	LOG_DBG("rssi_check for device %d with RSSI %d", conn_handle, rssi_new);
	// Mittelwert der letzten 10 Werte errechnen
    m_rssi_avg[conn_handle] = movingAvg(m_rssi_values[conn_handle], &m_rssi_sum[conn_handle], m_rssi_pos[conn_handle], RSSI_AVG_COUNT, rssi_new);
    
//    LOG_DBG("The new average is %d", m_rssi_avg[conn_handle]);
//    LOG_DBG("sum = %d", m_rssi_sum[conn_handle]);
//    
//    for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
//    {
//        LOG_RAW_DBG("arrNumbers[%d] = [", i);
//        for(uint8_t j = 0; j < RSSI_AVG_COUNT; j++)
//        {
//            LOG_RAW_DBG("%d, ", m_rssi_values[i][j]);
//        }
//        LOG_RAW_DBG("]\r\n");
//    }
    
    m_rssi_pos[conn_handle]++;
    if (m_rssi_pos[conn_handle] >= RSSI_AVG_COUNT){
        m_rssi_pos[conn_handle] = 0;
    }
  
    //LOG_DBG("RSSI avg from device %d = %d, threshold = %d", conn_handle, m_rssi_avg[conn_handle], m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold);
    
    // Aktuellen Zustand ermitteln
    uint8_t rssi_state = m_rssi_user_state[conn_handle];
    
    // Puffer zwischen Nah und Fern, da es sonst an der Grenze zu nicht gewünschtem Verhalten kommen kann
    // TODO: Prüfen, ob der Puffer groß genug ist
    if(m_rssi_avg[conn_handle] < m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_open)
        rssi_state = RSSI_STATE_NEAR;
    else if(m_rssi_avg[conn_handle] > m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_close)
        rssi_state = RSSI_STATE_FAR;
    
    if(rssi_state != m_rssi_user_state[conn_handle])
    {
        LOG_DBG("RSSI-Ergebnis - %s", rssi_str(rssi_state));
        LOG_DBG("RSSI avg from device %d = %d, threshold = %d", conn_handle, m_rssi_avg[conn_handle], m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_open);
        
        LOG_DBG("auto open = %d", m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].open_active);
        LOG_DBG("auto close = %d", m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].close_active);
        LOG_DBG("m_current_locking_state = %d", m_current_locking_state);
    }

    // Ergebnis auswerten
    // Übergang von Fern zu Nah, noch nicht begonnen Farbcode einzugeben und Motor nicht aktiv
    if(m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].open_active
        && m_dnd_mode_active == false && m_rssi_user_state[conn_handle] == RSSI_STATE_FAR 
        && rssi_state == RSSI_STATE_NEAR && m_current_locking_state == STATUS_MOTOR_1_CLOSED 
        && m_colorcode_in_index == 0 && ili_motorcontroller_get_state() == MOTOR_STOP)
    {
        set_motion_detection(false);
    }
    // Nutzer entfernt sich vom NEO -> Abschließen mit Bewegungsprüfung
    else if(m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].close_active
        && m_dnd_mode_active == false && m_rssi_user_state[conn_handle] == RSSI_STATE_NEAR 
        && rssi_state == RSSI_STATE_FAR && m_current_locking_state == STATUS_MOTOR_1_OPENED 
        && ili_motorcontroller_get_state() == MOTOR_STOP)
    {
        // Bewegung prüfen
        if(accelerometer_check(250) == ACC_NO_MOVEMENT)
        {
            // Falls keine Antwort vom Smartphone kommt, wird nach 3 Sek. geschlossen
            k_work_schedule(&work_app_movement_timeout, THREE_SEC_TIMEOUT_INTERVAL);
            // Anfrage an App senden, die Bewegung des Smartphones zu prüfen
            send_status(conn_handle, STATUS_APP_MOVEMENT);
        }
    }
    
    // Aktuellen Zustand setzen
    m_rssi_user_state[conn_handle] = rssi_state;
}


static void read_conn_rssi(uint16_t handle, uint8_t *rssi)
{
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	int err;
    k_timeout_t timeout = {.ticks = 100};

	buf = bt_hci_cmd_alloc(timeout);
	if (!buf) {
		LOG_ERR("Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_read_rssi *)rsp->data)->status : 0;
		LOG_ERR("Read RSSI err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	*rssi = comp8_t(rp->rssi);

    //LOG_DBG("RSSI: %d", *rssi);

	net_buf_unref(rsp);
}

/*
 * Thread Funktion zum Auslesen der RSSI-Werte
 */
void get_rssi()
{
    uint8_t rssi = 0;
    uint16_t conn_handle;
    LOG_DBG("RSSI-Thread started");

    while(1)
    {
        // Abarbeitung mittels Mutex schützen
        k_mutex_lock(&m_rssi_mutex, K_FOREVER);
        
        for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
        {
            if(m_rssi_active && m_connected_peer[i].used && m_auth_state[m_connected_peer[i].index] == AUTHORISED)
            {
                if(bt_hci_get_conn_handle(m_connected_peer[i].conn, &conn_handle) == APP_SUCCESS)
                {
                    //LOG_DBG("m_connected_peer[%d].index = %d : conn_handle = %d", i, m_connected_peer[i].index, conn_handle);
                    read_conn_rssi(conn_handle, &rssi);
                    rssi_check(m_connected_peer[i].index, rssi);
                }
                //else
                //{
                //    LOG_DBG("conn_handle wurde nicht gefunden.");
                //}
            }
        }

        // Mutex freigeben
        k_mutex_unlock(&m_rssi_mutex);

        k_msleep(50);
    }
}


static void app_gdio_data_rx_cb(struct bt_conn* conn, uint8_t* data, uint8_t len)
{
	LOG_HEXDUMP_DBG(data, len, "GDIO-Data");

	peer_context_t* peripheral = get_peer_context_from_conn(conn);
	
	if(peripheral)
	{
		uint32_t err_code = ili_receive_gdio_msg(peripheral->index, data, len, m_app_msg_buffer[peripheral->index], &m_app_msg_buffer_length[peripheral->index]);
		LOG_INF("ili_receive_gdio_msg() = %d", err_code);
				if(err_code == PARSER_MESSAGE_COMPLETE)
				{
					// Festlegen, von welcher Verbindung die Nachricht stammt
					m_gdio_data_received |= (1 << peripheral->index);
				}
	}
	else
		LOG_WRN("Connection not found");
}


void app_gdio_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
    LOG_DBG("app_gdio_indicate_cb");
    m_gdio_indication_ack = true;
}


static void app_usdio_data_rx_cb(struct bt_conn* conn, uint8_t* data, uint8_t len)
{
	LOG_HEXDUMP_DBG(data, len, "USDIO-Data");
    
	peer_context_t* peripheral = get_peer_context_from_conn(conn);
    if(peripheral)
	{
		uint32_t err_code = ili_receive_usdio_msg(peripheral->index, data, len, m_app_msg_buffer[peripheral->index], &m_app_msg_buffer_length[peripheral->index]);
		LOG_INF("ili_receive_usdio_msg() = %d", err_code);
		if(err_code == PARSER_MESSAGE_COMPLETE)
		{
			m_usdio_data_received |= (1 << peripheral->index);
		}
	}
	else
		LOG_WRN("Connection not found");
}


void app_usdio_indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
    LOG_DBG("app_usdio_indicate_cb");
    m_usdio_indication_ack = true;
}


/**
 * @brief Senden einer Status-Mitteilung
 */
void send_status(uint16_t conn_handle, uint8_t status)
{
    LOG_DBG("send_status 0x%.2X to peer %d", status, conn_handle);

    uint8_t idx_begin = 0;
    uint8_t idx_end = 0;

    // Indizes für Schleife festelegen
    if(conn_handle == BLE_CONN_HANDLE_ALL)
    {
        idx_end = CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT;
    }
    else
    {
        idx_begin = conn_handle;
        idx_end = conn_handle + 1;
    }
    
    //LOG_DBG("idx_begin = %d", idx_begin);
    //LOG_DBG("idx_end = %d", idx_end);
    

    // Statusmeldung(en) in Queue einfügen
    for(uint8_t i = idx_begin; i < idx_end; i++)
    {
        //LOG_DBG("i = %d", i);
        if(m_connected_peer[i].used)
        {
            //LOG_DBG("is used");
            if((m_auth_state[m_connected_peer[i].index] == AUTHORISED || m_bonding_mode_active
                || status == STATUS_ERR_NO_AUTH || status == STATUS_ERR_AUTH_FAILED || status == STATUS_ERR_GENERAL || status == STATUS_ERR_NOT_ALLOWED))
            {
                status_t msg = {
                    .conn_handle = m_connected_peer[i].index,
                    .status_code = status
                };

                LOG_DBG("status added in Queue");
                if (k_is_in_isr()) {
                    // ISR-Kontext → niemals blockieren
                    k_msgq_put(&m_status_msgq, &msg, K_NO_WAIT);
                } else {
                    // Thread-Kontext → optional blockierend
                    k_msgq_put(&m_status_msgq, &msg, K_FOREVER);
                }
            }
        }
    }
}


TD("conn_handle am besten umbenennen, um Verwirrung zu vermeiden. Es handelt sich um den Index des Peer Context")
void gdio_data_received(uint16_t conn_handle)
{
    LOG_DBG("gdio_data_received() from device %d", conn_handle);
    uint32_t err_code = ili_parse_gdio_msg(m_app_msg_buffer[conn_handle], m_app_msg_buffer_length[conn_handle], &m_gdio_message_in[conn_handle]);
//    LOG_DBG("ili_parse_app_msg = %d", err_code);
//    LOG_DBG("m_app_message.command = %d", m_gdio_message_in[conn_handle].command);
//    LOG_DBG("m_app_message.payload_length = %d", m_gdio_message_in[conn_handle].payload_len);
    
    switch(err_code)
    {
        case PARSER_ERROR_INVALID_LENGTH:
        {
            send_status(conn_handle, STATUS_ERR_GENERAL);
            LOG_DBG("Falsche Nachrichtengröße");
            return;
        }
        
        case PARSER_ERROR_WRONG_CRC:
        {
            send_status(conn_handle, STATUS_ERR_CRC);
            LOG_DBG("CRC nicht korrekt");
            return;
        }
    }
    
    ili_gdio_message_t app_message_out;
    
    switch(m_gdio_message_in[conn_handle].command)
    {
        // Anfrage von Daten an die App
        case REQUEST_DATA:
        {
            memcpy(&app_message_out.command, &m_gdio_message_in[conn_handle].payload, 2);
//            LOG_DBG("REQUEST_DATA: %d", app_message_out.command);
            // Angeforderte Daten senden
            gdio_send_data(conn_handle, &app_message_out);
        }
        break; // case REQUEST_DATA
        
        case AUTH_AUTHENTICATOR:
        {
            if(m_auth_state[conn_handle] == CHALLENGE_REQUESTED && m_bonding_mode_active)
            {
                LOG_HEXDUMP_DBG(m_stk, LTK_LEN, "STK:");
                LOG_HEXDUMP_DBG(m_nonce[conn_handle], SHA256_LENGTH, "Nonce:");
                
                // Authentifizierungs-Token berechnen
                uint8_t	authenticator[SHA256_LENGTH];// Authentifizierungs-Token
                /* Setup a multipart hash operation */
				psa_hash_setup(&m_sha256, PSA_ALG_SHA_256);
                psa_hash_update(&m_sha256, m_stk, LTK_LEN);                
                psa_hash_update(&m_sha256, m_nonce[conn_handle], SHA256_LENGTH);
                psa_hash_finish(&m_sha256, authenticator, sizeof(authenticator), &m_sha256_len);
                
                LOG_HEXDUMP_DBG(authenticator, SHA256_LENGTH, "Authentifizierungs-Token:");
                
                // Mit dem selbst berechneten Authentifizierungs-Token vergleichen
                if(memcmp(&m_gdio_message_in[conn_handle].payload, authenticator, SHA256_LENGTH) == 0)
                {
                    LOG_DBG("Authentifizierungs-Token korrekt");
                    
                    m_auth_state[conn_handle] = TOKEN_CORRECT;
                    app_message_out.command = CHALLENGE;
                                        
                    // Sende Challenge
                    gdio_send_data(conn_handle, &app_message_out);
                }
                else
                {
                    m_auth_state[conn_handle] = UNAUTHORISED;
                    send_status(conn_handle, STATUS_ERR_BOND_FAILED);
                }
            }
            else
            {
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
                LOG_DBG("Falscher Zustand");
                break;
            }
        }
        break; // case AUTH_AUTHENTICATOR
        
        case AUTH_DATA:
        {
            if(m_auth_state[conn_handle] == TOKEN_CHALLENGE_SENT && m_bonding_mode_active)
            {
				// Authentifizierungs-Token berechnen
				uint8_t	authenticator[SHA256_LENGTH];// Authentifizierungs-Token
				
                /* Setup a multipart hash operation */
				psa_hash_setup(&m_sha256, PSA_ALG_SHA_256);
                psa_hash_update(&m_sha256, &m_gdio_message_in[conn_handle].payload[SHA256_LENGTH], SHA256_LENGTH+HW_ID_LEN);
                psa_hash_update(&m_sha256, m_nonce[conn_handle], SHA256_LENGTH);
				psa_hash_finish(&m_sha256, authenticator, sizeof(authenticator), &m_sha256_len);
                
                // Mit dem selbst berechneten Authentifizierungs-Token vergleichen
                if(memcmp(m_gdio_message_in[conn_handle].payload, authenticator, SHA256_LENGTH) != 0)
                {
                    LOG_DBG("Authentifizierungs-Token nicht korrekt");
        
                    m_auth_state[conn_handle] = UNAUTHORISED;
                    send_status(conn_handle, STATUS_ERR_BOND_FAILED);
                    break;
                }
                
                m_auth_state[conn_handle] = AUTH_DATA_CORRECT;
                    
                // Daten temporär speichern und Authentifizierungs-ID festlegen
                // Peer-ID speichern und aktuellen Nutzer festlegen
                if(add_peer_data(&m_gdio_message_in[conn_handle].payload[SHA256_LENGTH]) != APP_SUCCESS)
                {
                    LOG_DBG("Fehler beim Hinzufügen eines neuen Nutzers!");
            
                    m_auth_state[conn_handle] = UNAUTHORISED;
                    send_status(conn_handle, STATUS_ERR_BOND_FAILED);
                    break;
                }
                LOG_DBG("Peer hinzugefügt");
                     
                // Authorization-ID senden (AUTH_ID)
                psa_generate_random(m_nonce[conn_handle], SHA256_LENGTH);

				/* Setup a multipart hash operation */
				psa_hash_setup(&m_sha256, PSA_ALG_SHA_256);
                psa_hash_update(&m_sha256, &m_connected_peer[conn_handle].peer_data->auth_id, 1);
                psa_hash_update(&m_sha256, m_nonce[conn_handle], SHA256_LENGTH);
                psa_hash_update(&m_sha256, &m_gdio_message_in[conn_handle].payload[SHA256_LENGTH+HW_ID_LEN], SHA256_LENGTH);
				// Authenticator berechnen
				psa_hash_finish(&m_sha256, authenticator, sizeof(authenticator), &m_sha256_len);                
                    
                app_message_out.command = AUTH_ID;
                memcpy(app_message_out.payload, authenticator, SHA256_LENGTH);
                memcpy(&app_message_out.payload[SHA256_LENGTH], &m_connected_peer[conn_handle].peer_data->auth_id, 1);
                memcpy(&app_message_out.payload[SHA256_LENGTH+1], m_nonce[conn_handle], SHA256_LENGTH);
                app_message_out.payload_len = SHA256_LENGTH+SHA256_LENGTH+1;
                
                // Sende Auth-ID
                gdio_send_data(conn_handle, &app_message_out);	
            }
            else
            {
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
                LOG_DBG("Falscher Zustand");
                break;
            }
            
        }
        break; // case AUTH_DATA
        
        case AUTH_ID_CONF:
        {
            if(m_auth_state[conn_handle] == AUTH_DATA_CORRECT && m_bonding_mode_active)
            {
                // Authenticator berechnen
                uint8_t	authenticator[SHA256_LENGTH];// Authentifizierungs-Token
				/* Setup a multipart hash operation */
				psa_hash_setup(&m_sha256, PSA_ALG_SHA_256);
                /* Feed the chunks of the input data to the PSA driver */
				psa_hash_update(&m_sha256, &m_connected_peer[conn_handle].peer_data->auth_id, 1);
				psa_hash_update(&m_sha256, m_nonce[conn_handle], SHA256_LENGTH);
				// Authenticator berechnen
				psa_hash_finish(&m_sha256, authenticator, sizeof(authenticator), &m_sha256_len);
                
                // Authenticator vergleichen
                if(memcmp(m_gdio_message_in[conn_handle].payload, authenticator, SHA256_LENGTH) != 0)
                {
//                    clear_peer_data();
                    m_auth_state[conn_handle] = UNAUTHORISED;
                    send_status(conn_handle, STATUS_ERR_BOND_FAILED);
                    LOG_DBG("Authenticator nicht korrekt!");
                    LOG_DBG("Disconnect");
                    break;
                }
                
                LOG_DBG("Authenticator korrekt!");
                if(m_connected_peer[conn_handle].peer_data->auth_id != m_gdio_message_in[conn_handle].payload[SHA256_LENGTH])
                {
//                    clear_peer_data();
                    m_auth_state[conn_handle] = UNAUTHORISED;
                    send_status(conn_handle, STATUS_ERR_BOND_FAILED);
                    LOG_DBG("Auth-ID nicht korrekt!");
                    LOG_DBG("Disconnect");
                    break;
                }
                
                LOG_DBG("Auth-ID korrekt!");
                m_auth_state[conn_handle] = AUTH_ID_CORRECT;

                // Befehl erstellen
                app_message_out.command = IV;
                                
                gdio_send_data(conn_handle, &app_message_out);
                
                // Zustand aktualisieren
                m_auth_state[conn_handle] = IV_SENT;
            }
            else
            {
//                clear_peer_data();
                m_auth_state[conn_handle] = UNAUTHORISED;
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
                LOG_DBG("AUTH_ID_CONF Disconnect");
                break;
            }
        }
        break; // case AUTH_ID_CONF
     
    } // switch(m_command)
}


void gdio_send_data(uint16_t conn_handle, ili_gdio_message_t* message_out)
{
    LOG_DBG("gdio_send_data() Command: 0x%.4X to device %d", message_out->command, conn_handle);

    // Nichts senden, wenn kein Geräte verbunden ist
    static struct bt_conn_info conn_info;
    bt_conn_get_info(m_connected_peer[conn_handle].conn, &conn_info);

    if(m_peripheral_conn_count == 0 || conn_info.state != BT_CONN_STATE_CONNECTED)
    {
        LOG_DBG("Statusmeldung soll an nicht verbundenes Gerät gesendet werden");
		return;
    }


    LOG_DBG("m_auth_state[conn_handle] = %d", m_auth_state[conn_handle]);
    if(m_connected_peer[conn_handle].used == false)
    {
        LOG_DBG("m_connected_peer[conn_handle] == NULL");
    }
    else
    {
        LOG_DBG("m_connected_peer[conn_handle] == NOT NULL");
    }
    
    switch(message_out->command)
	{
        case IV:
		{
			if((m_bonding_mode_active && m_auth_state[conn_handle] != AUTH_ID_CORRECT))
			{
				LOG_DBG("IV: Falscher Zustand");
				send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
				return;
			}
			else if(m_bonding_mode_active == false && m_auth_state[conn_handle] != UNAUTHORISED)
			{
				LOG_DBG("IV: Peer-Daten zurücksetzen");
				
				// Peer-Daten ungültig machen, Autorisierung beginnt von vorn
				m_connected_peer[conn_handle].peer_data = NULL;
				m_auth_state[conn_handle] = UNAUTHORISED;
				
				// Schließzähler zurücksetzen
				m_lock_counter[conn_handle] = 0;
			}
			
			// Initialisierungsvektor erzeugen
			psa_generate_random(message_out->payload, IV_LENGTH);
            
            LOG_HEXDUMP_DBG(message_out->payload, 16, "IV");
			
			// Befehl erstellen
			message_out->payload_len = IV_LENGTH;
            
            // Verschlüsselung initialisieren
            if(m_bonding_mode_active)
            {
                ili_set_key(conn_handle, m_stk);
            
                LOG_HEXDUMP_DBG(m_stk, LTK_LEN, "STK-KEY");
            }
            
            ili_set_iv(conn_handle, message_out->payload);
		}
		break; // case IV
        
        case SERIALNUMBER:
		{
            LOG_DBG("SERIALNUMBER %d", m_uicr_data.serial_number);
			// Befehl erstellen
			memcpy(message_out->payload, &m_uicr_data.serial_number, 4);
			message_out->payload_len = 4;
		}
		break; // case SERIALNUMBER
        
        case CHALLENGE:
		{
			if(m_bonding_mode_active && (m_auth_state[conn_handle] == TOKEN_CORRECT || (m_auth_state[conn_handle] == UNAUTHORISED && m_peer_started_bonding == BLE_CONN_HANDLE_INVALID)))
			{
				psa_generate_random(m_nonce[conn_handle], SHA256_LENGTH);
                        
				// Befehl erstellen
				memcpy(message_out->payload, m_nonce[conn_handle], SHA256_LENGTH);
				message_out->payload_len = SHA256_LENGTH;

				if(m_auth_state[conn_handle] == UNAUTHORISED)
                {
					m_auth_state[conn_handle] = CHALLENGE_REQUESTED;
                    // Bonding-Modus für andere Teilnehmer blockieren
                    m_peer_started_bonding = conn_handle;
				}
                else if(m_auth_state[conn_handle] == TOKEN_CORRECT)
					m_auth_state[conn_handle] = TOKEN_CHALLENGE_SENT;
			}
			else
			{
				send_status(conn_handle, STATUS_ERR_GENERAL);
				LOG_DBG("Falscher Zustand");
				return;
			}
		}
		break; // case CHALLENGE
        
        case CHALLENGE_GPS:
		{
			if(m_bonding_mode_active && m_peer_started_bonding == BLE_CONN_HANDLE_INVALID && m_auth_state[conn_handle] == UNAUTHORISED)
			{
				psa_generate_random(m_nonce[conn_handle], SHA256_LENGTH);
				
				// Befehl erstellen
				// Befehlsnummer anpassen, um Challenge zu senden
                message_out->command = CHALLENGE;
				memcpy(message_out->payload, m_nonce[conn_handle], SHA256_LENGTH);
				message_out->payload_len = SHA256_LENGTH;

				if(m_auth_state[conn_handle] == UNAUTHORISED)
                {
					m_auth_state[conn_handle] = CHALLENGE_REQUESTED;
                    // Bonding-Modus für andere Teilnehmer blockieren
                    m_peer_started_bonding = conn_handle;
				}
                else if(m_auth_state[conn_handle] == TOKEN_CORRECT)
					m_auth_state[conn_handle] = TOKEN_CHALLENGE_SENT;
			}
			else
			{
				send_status(conn_handle, STATUS_ERR_GENERAL);
				LOG_DBG("Falscher Zustand");
				return;
			}
		}
		break; // case CHALLENGE_GPS

        case LOCK_STATUS:
		{			
			LOG_DBG("Lock Status: 0x%.2X", message_out->payload[0]);
		}
		break; // case LOCK_STATUS
		
        case AUTH_ID:
            
        break;
        
		default:
			send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
			LOG_DBG("Unbekannter Befehl");
			return;
    }
    
    // Daten senden
	uint8_t index = 0;
    uint8_t* msg = (uint8_t*)message_out;
    uint16_t crc = crc16_itu_t(CRC_SEED, msg, message_out->payload_len + 2);
//    LOG_DBG("CRC byte array: %.2X", crc);
    
    // CRC anfügen
    memcpy(&message_out->payload[message_out->payload_len], &crc, 2);
    message_out->payload_len += GDIO_HEADER_SIZE;
    
    uint8_t length = message_out->payload_len;
   
    LOG_HEXDUMP_DBG(msg, length, "Sende GDIO:");
  
	while(length > 0 && conn_info.state == BT_CONN_STATE_CONNECTED)
	{
		m_gdio_indication_ack = false;
		
		uint16_t bytes_to_send = MIN(length, 20);
        
        k_timer_start(&m_gdio_ind_ack_timer, IND_ACK_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);

        // conn_handle muss hier um NRF_SDH_BLE_CENTRAL_LINK_COUNT erhöht werden, damit an das richtige Handle gesendet wird
        uint32_t err_code = ili_send_gdio_ind(m_connected_peer[conn_handle].conn, &msg[index], bytes_to_send);
		
        				
        LOG_DBG("ble_ili_gdio_data_send = %d", err_code);
		if(err_code == 0)
        {
            // Warten bis Acknowledge eingetroffen		
            while(m_gdio_indication_ack == false)
            {
                k_msleep(50);
            }
        }

		k_timer_stop(&m_gdio_ind_ack_timer);
        
		length -= bytes_to_send;
		index += bytes_to_send;
	}
    
    // Am Ende des Werkszustands neustarten
	if(message_out->command == LOCK_STATUS && message_out->payload[0] == STATUS_FACTORY_RESET_COMPLETE)
	{
        LOG_DBG("NVIC_SystemReset");
        k_sleep(K_MSEC(1000));
        NVIC_SystemReset();
	}
}


void usdio_data_received(uint16_t conn_handle)
{
    LOG_DBG("usdio_data_received()");
    
    // Erste verschlüsselte Nachricht vom Peer
    if(m_connected_peer[conn_handle].peer_data == NULL)
    {
        uint8_t auth_id = m_app_msg_buffer[conn_handle][0];
        if(auth_id < MAX_PEER_COUNT && m_peer_data[auth_id].is_used)
        {
            LOG_DBG("Auth-ID gesetzt");
            m_connected_peer[conn_handle].peer_data = &m_peer_data[auth_id];
            
            // Verschlüsselung initialisieren
            
            // LTK aus dem Secure Storage laden
            size_t key_len;

            psa_status_t status = psa_export_key(ZEPHYR_PSA_APPLICATION_KEY_ID_RANGE_BEGIN + auth_id, m_connected_peer[conn_handle].ltk, 32, &key_len);
            LOG_DBG("-----------psa_export_key = %d", status);
            LOG_DBG("-----------Exported Key length = %d", key_len);
            LOG_HEXDUMP_DBG(m_connected_peer[conn_handle].ltk, key_len, "Key Data: ");
    
            if(key_len == LTK_LEN)
                ili_set_key(conn_handle, m_connected_peer[conn_handle].ltk);
        }
        else
        {
            if(m_auth_state[conn_handle] == AUTHORISED)
                send_status(conn_handle, STATUS_ERR_WRONG_AUTH_ID);
            else
                send_status(conn_handle, STATUS_ERR_GENERAL);
            
            LOG_DBG("Auth-ID nicht gefunden");
            return;
        }
    }
    
    uint32_t err_code = ili_parse_usdio_msg(conn_handle, m_app_msg_buffer[conn_handle], m_app_msg_buffer_length[conn_handle], &m_usdio_message_in[conn_handle]);
//    LOG_DBG("ili_parse_app_msg = %d", err_code);
//    LOG_DBG("m_app_message.command = %d", m_usdio_message_in[conn_handle].command);
//    LOG_DBG("m_app_message.payload_length = %d", m_usdio_message_in[conn_handle].payload_len);
    
    switch(err_code)
    {
        case PARSER_ERROR_INVALID_LENGTH:
        {
            if(m_auth_state[conn_handle] == AUTHORISED)
                send_status(conn_handle, STATUS_ERR_WRONG_MSG_SIZE);
            else
                send_status(conn_handle, STATUS_ERR_GENERAL);
            
            return;
        }
        
        case PARSER_ERROR_WRONG_CRC:
        {
            if(m_auth_state[conn_handle] == AUTHORISED)
                send_status(conn_handle, STATUS_ERR_CRC);
            else if(m_bonding_mode_active)
            {
                m_connected_peer[conn_handle].peer_data = NULL;
                ili_reset_parser(conn_handle);
                m_auth_state[conn_handle] = UNAUTHORISED;
                send_status(conn_handle, STATUS_ERR_BOND_FAILED);
            }
            else
            {
                m_auth_state[conn_handle] = UNAUTHORISED;
                send_status(conn_handle, STATUS_ERR_AUTH_FAILED);
            }
            LOG_DBG("ERROR: CRC falsch");
            
            return;
        }
        
        case PARSER_ERROR_INVALID_AUTHID:
        {
            if(m_auth_state[conn_handle] == AUTHORISED)
                send_status(conn_handle, STATUS_ERR_WRONG_AUTH_ID);
            else
                send_status(conn_handle, STATUS_ERR_GENERAL);
            
            return;
        }
    }
    
    ili_usdio_message_t app_message_out;
    
    if(m_auth_state[conn_handle] != AUTHORISED && m_usdio_message_in[conn_handle].command != REQUEST_DATA 
        && m_usdio_message_in[conn_handle].command != CHALLENGE_RESP && m_usdio_message_in[conn_handle].command != LTK_CONF)
	{
		send_status(conn_handle, STATUS_ERR_NO_AUTH);
		LOG_DBG("Nicht autorisiert");
		return;
	}
    
    switch(m_usdio_message_in[conn_handle].command)
    {
        case REQUEST_DATA:
        {
            memcpy(&app_message_out.command, &m_usdio_message_in[conn_handle].payload, 2);
//            LOG_DBG("REQUEST_DATA: %d", app_message_out.command);
            // Angeforderte Daten senden
            usdio_send_data(conn_handle, &app_message_out);
        }
        break; // case REQUEST_DATA
        
        case CHALLENGE_RESP:
        {
            LOG_DBG("CHALLENGE_RESP");
            
            if(memcmp(&m_usdio_message_in[conn_handle].payload, m_nonce[conn_handle], SHA256_LENGTH) == 0 
                && memcmp(&m_usdio_message_in[conn_handle].payload[SHA256_LENGTH], m_connected_peer[conn_handle].peer_data->hw_id, HW_ID_LEN) == 0)
            {
                if(m_bonding_mode_active)
                {
                    // Mit Temp-Key verschlüsselten LTK senden
                    // Anschließend wird nur noch der LTK genutzt (mit ursprünglichem IV)
                    m_auth_state[conn_handle] = CHALL_RESP_CORRECT;
                    
                    // LTK-Seed zufällig festlegen
                    psa_generate_random(m_nonce[conn_handle], SHA256_LENGTH);

                    // Long Term Key berechnen
                    uint8_t zeros[16];
                    memset(zeros, 0, 16);
                    unsigned char sigma[16] = "ILockIt Plus LTK";
             
                    /* Setup a multipart hash operation */
                    psa_hash_setup(&m_sha256, PSA_ALG_SHA_256);
                    /* Feed the chunks of the input data to the PSA driver */
                    psa_hash_update(&m_sha256, m_nonce[conn_handle], SHA256_LENGTH);
                    psa_hash_update(&m_sha256, zeros, 16);
                    psa_hash_update(&m_sha256, sigma, 16);
                    // LTK ausgeben
                    psa_hash_finish(&m_sha256, m_ltk, LTK_LEN, &m_sha256_len);
                
                    app_message_out.command = LTK;
                    memcpy(app_message_out.payload, m_nonce[conn_handle], SHA256_LENGTH);
                    app_message_out.payload_len = SHA256_LENGTH;
                    usdio_send_data(conn_handle, &app_message_out);
                    
                    memcpy(m_connected_peer[conn_handle].ltk, m_ltk, LTK_LEN);
                    ili_set_key(conn_handle, m_connected_peer[conn_handle].ltk);
                    
                    LOG_HEXDUMP_DBG(m_connected_peer[conn_handle].ltk, LTK_LEN, "LTK:");

                    m_auth_state[conn_handle] = LTK_SENT;
                }
                else
                {
                    LOG_DBG("Nutzer wurde authorisiert!");
                    m_auth_state[conn_handle] = AUTHORISED;
                    			
                    // Blaue LED anschalten
                    led_timed(LED_B, LED_STATIC);
                    
                    // RSSI Abfrage starten
                    rssi_start();
               
                    // Wenn Alarm aktiv ist, dann die Notification nach Service Abonnierung übertragen
                    if (m_alarmsound_active)
                        send_status(conn_handle, STATUS_ALARM_ON);
                    
                    // App melden, dass der Verbingungsaufbau erfolgeich war
                    send_status(conn_handle, STATUS_CONN_COMPLETE);
                }
            }
            else
            {
                // Peer-Daten ungültig machen
                m_connected_peer[conn_handle].peer_data = NULL;
                m_auth_state[conn_handle] = UNAUTHORISED;
                ili_reset_parser(conn_handle);
                if(m_bonding_mode_active)
                {
 //                   clear_peer_data();
                    send_status(conn_handle, STATUS_ERR_BOND_FAILED);
                }
                else
                {
                    // Fehlercode an App übergeben, dass Autorisierung fehlgeschlagen
                    send_status(conn_handle, STATUS_ERR_AUTH_FAILED);
                }
                LOG_DBG("ERROR: Challenge Response falsch");
            }
        }
        break; // case CHALLENGE_RESP
        
        case LTK_CONF:
        {
            LOG_DBG("LTK_CONF");
            if(m_auth_state[conn_handle] == LTK_SENT)
            {
                LOG_DBG("Nutzer wurde authorisiert!");
                LOG_DBG("m_num_app_bonds = %d", m_num_app_bonds);
                
                m_auth_state[conn_handle] = AUTHORISED;
                
                // Lock-Status BOND_COMPLETE senden
                send_status(conn_handle, STATUS_BOND_COMPLETE);
        
                // Wenn bereits eingerichtet, blaues Blinken beenden
                // Farbcode muss hier nicht mehr übertragen werden
                if(m_num_app_bonds > 0 || m_num_fob_bonds > 0)
                {
                    m_bonding_mode_active = false;

                    k_timer_stop(&m_bond_timer);
                    // Blaue LED anschalten
                    led_timed(LED_B, LED_STATIC);
                    
                    m_connected_peer[conn_handle].peer_data->is_used = true;
                    m_update_peer_auth_id = m_connected_peer[conn_handle].peer_data->auth_id;
                    LOG_DBG("Peer hinzugefügt");
                    
                    
                    // Scan neustarten, falls HS vorhanden
                    if (m_num_fob_bonds > 0 && m_fob_conn == NULL)
                    {
                        bt_scan_stop();
//                        fob_update_in_progress = false;
                        LOG_DBG("LTK_CONF - start scanning");
                        scan_start();
                    }
                }
               
                // RSSI Abfrage starten
                rssi_start();
                
                // Wenn Alarm aktiv ist, dann die Notification nach Service Abonnierung übertragen
                if (m_alarmsound_active)
                    send_status(conn_handle, STATUS_ALARM_ON);
            
            }
            else
            {
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
            }
        }
        break; // case LTK_CONF

				
        case LOCK_ACTION:
        {
            LOG_DBG("LOCK_ACTION");
            
        	// Überprüfung des Schließzählers
            if(memcmp(m_usdio_message_in[conn_handle].payload, &m_lock_counter[conn_handle], 2) > 0)
            {
                memcpy(&m_lock_counter[conn_handle], m_usdio_message_in[conn_handle].payload, 2);
                LOG_DBG("m_lock_counter = %d", m_lock_counter[conn_handle]);
            }
            else
            {
                
                // Fehlermeldung ausgeben und Schließvorgang abbrechen
                send_status(conn_handle, STATUS_ERR_WRONG_COUNTER);
                LOG_DBG("STATUS_ERR_WRONG_COUNTER");
                break;
            }
            
            // Befehlsargument auswerten
            switch(m_usdio_message_in[conn_handle].payload[2])
            {
                LOG_DBG("Lock cmd = %d", m_usdio_message_in[conn_handle].payload[2]);
                // Aufschließen
                case LOCK_ACTION_DISABLE_ALARM:
                {
                    set_motion_detection(false);
                }
                break;
                
                // Zuschließen
                case LOCK_ACTION_ENABLE_ALARM:
                {
                    set_motion_detection(true);
                }
                break;

                // Motor 2 aufschließen
                case LOCK_ACTION_OPEN_CHAIN:
                {
                    alarmcheck_stop();
                    led_timed(LED_G, LED_UNLOCKING);
                    ili_motorcontroller_start(true);
                }
                break;
                
                // Motor 2 zuschließen
                case LOCK_ACTION_CLOSE_CHAIN:
                {
                    alarmcheck_stop();
                    led_timed(LED_R, LED_LOCKING);
                    ili_motorcontroller_start(false);
                }
                break;

                case LOCK_ACTION_APP_MOVEMENT_NO:
                {
                    // Keine Bewegung am Smartphone -> Autom. Schließen abbrechen
                    k_work_cancel_delayable(&work_app_movement_timeout);
                }break;

                case LOCK_ACTION_APP_MOVEMENT_YES:
                {
                    // Bewegung am Smartphone -> Abschließen
                    k_work_cancel_delayable(&work_app_movement_timeout);
                    set_motion_detection(true);
                }break;
                
                default:
                {
                    // Fehlercode an App senden
                    send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
                }
                break;
            } //switch(m_usdio_message_in[conn_handle].payload[2])
		}
        break; // case LOCK_ACTION
        
        case DEVICE_SETTINGS:
        {
            LOG_DBG("DEVICE_SETTINGS %d", m_usdio_message_in[conn_handle].payload[0]);
            
            switch(m_usdio_message_in[conn_handle].payload[0])
            {
                // Werkszustand herstellen
                case DEVICE_SETTINGS_FACTORY_RESET:
                {                
                    // Wenn eine Verbindung zum HS+ besteht, wird diese getrennt
                    if (m_fob_conn != NULL)
                    {
                        // Disconnect from peer.
                        bt_conn_disconnect(m_fob_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                    }
                
                    if(m_current_locking_state_chain == STATUS_MOTOR_2_OPENED && m_current_locking_state == STATUS_MOTOR_1_OPENED)
                    {
                        // Alle Bonding-Datensätze löschen
                        m_start_factory_reset = true;
                    }
                    else
                    {
                        send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
                    }
                }
                break;
                
                case DEVICE_SETTINGS_NEW_BOND:
                {
                    //LOG_DBG("Paring-Mode starten");
                    new_bond();
                }
                
                // Aktiven Alarm deaktivieren
                case DEVICE_SETTINGS_QUIT_ALARM:
                {
                    if(m_alarmsound_active)
                    {
                        // Alarm ausschalten
                        all_sounds_stop();     
                    
                        // Einige Sekunden warten, bis der Alarm wieder aktiviert wird
                        k_timer_start(&m_alarm_restart_timer, FIVE_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
                    }
                }
                break;
            }
        }
        break; // case DEVICE_SETTINGS
        
        case CLICK_CODE:
		{
            LOG_DBG("CLICK_CODE %d-%d-%d", m_usdio_message_in[conn_handle].payload[0], m_usdio_message_in[conn_handle].payload[1], m_usdio_message_in[conn_handle].payload[2]);
            
            if (m_usdio_message_in[conn_handle].payload[0] <= 33 && m_usdio_message_in[conn_handle].payload[1] <= 33 && m_usdio_message_in[conn_handle].payload[2] <= 33) 
            {
                // Prüfen, ob sich etwas geändert hat
                if(memcmp(m_settings.colorcode_flash, m_usdio_message_in[conn_handle].payload, 3) != 0)
                {
                    memcpy(m_settings.colorcode_flash, m_usdio_message_in[conn_handle].payload, 3);
                    
                    // Farbcode berechnen
                    set_colorcodes();
                    
                    LOG_DBG("click_code_bond = %d %d %d %d %d %d", m_colorcode[0], m_colorcode[1], m_colorcode[2], m_colorcode[3], m_colorcode[4], m_colorcode[5]);
                        
                    // Bei Ersteinrichtung wird das Flag für die Einrichtung 
                    // erst nach erfolgreichem Schreiben des Farbcodes gesetzt
                    if(m_num_app_bonds == 0)
                    {
                        LOG_DBG("m_write_click_code");
                        m_write_click_code = true;
                    }
                    
                    // Neuen Code speichern
                    m_update_settings = true;
                    LOG_DBG("update_settings");
                }
            }
            else
            {
                // Fehlercode an App senden
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
            }
        }
        break; // case CLICK_CODE
  
        case ALARM_SETTINGS:
        {
            LOG_DBG("ALARM_SETTINGS %d-%d", m_usdio_message_in[conn_handle].payload[0], m_usdio_message_in[conn_handle].payload[1]);
            // Alarmeinstellung setzen
            if (m_usdio_message_in[conn_handle].payload[0] <= 0x01 && m_dnd_mode_active == false)
            {
                // Prüfen, ob sich etwas geändert hat
                if(m_settings.alarm.armed != m_usdio_message_in[conn_handle].payload[0]
                    || m_settings.alarm.ble != m_usdio_message_in[conn_handle].payload[1])
                {
                    m_settings.alarm.armed = m_usdio_message_in[conn_handle].payload[0];
                    m_settings.alarm.ble = m_usdio_message_in[conn_handle].payload[1];
                
                    m_settings.alarm.prealarm = ((m_settings.alarm.ble & ALARM_MODE_PREALARM) != 0);
                    
                    if(m_settings.alarm.prealarm == false)
                        m_settings.alarm.silent = ((m_settings.alarm.ble & ALARM_MODE_SILENT) != 0);
                    else
                        m_settings.alarm.silent = false;
                    
                    // Schwellwert für Alarm festlegen
                    if(m_settings.alarm.ble & ALARM_MODE_HIGHEST)
                        m_settings.alarm.threshold = ALARM_SENS_HIGHEST;
                    else if(m_settings.alarm.ble & ALARM_MODE_HIGHER)
                        m_settings.alarm.threshold = ALARM_SENS_HIGHER;
                    else if(m_settings.alarm.ble & ALARM_MODE_HIGH)
                        m_settings.alarm.threshold = ALARM_SENS_HIGH;
                    else if(m_settings.alarm.ble & ALARM_MODE_MID)
                        m_settings.alarm.threshold = ALARM_SENS_MID;
                    else if(m_settings.alarm.ble & ALARM_MODE_LOW)
                        m_settings.alarm.threshold = ALARM_SENS_LOW;	    	
                    else if(m_settings.alarm.ble & ALARM_MODE_LOWEST)
                        m_settings.alarm.threshold = ALARM_SENS_LOWEST;  
                    
                    m_update_settings = true;
                    
                    // Alarm beenden, wenn deaktiviert
                    if(m_settings.alarm.armed == false)
                    {
                        all_sounds_stop();
                        alarmcheck_stop();
                    }
                    else
                    {
                        // Alarmauswertung neustarten
                        alarmcheck_stop();
                        alarmcheck_start();
                    }
                }
            }
            else if(m_dnd_mode_active)
            {
                LOG_DBG("Fehler - DND aktiv");
                // Fehlercode an App senden
                send_status(conn_handle, STATUS_ERR_DND_ACTIVE);
            }
            else
            {
                LOG_DBG("Fehler");
                // Fehlercode an App senden
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
            }
        }
        break; // case ALARM_SETTINGS
        
        case ALARM_SETTINGS_PRO:
        {
            // [0] - an/aus, [1] - 0/ALARM_MODE_SILENT/ALARM_MODE_PREALARM, [2] - m_settings.alarm.threshold
            LOG_DBG("ALARM_SETTINGS_PRO %d-%d-%d", m_usdio_message_in[conn_handle].payload[0], m_usdio_message_in[conn_handle].payload[1], m_usdio_message_in[conn_handle].payload[2]);
             // Alarmeinstellung setzen
            if (m_usdio_message_in[conn_handle].payload[0] <= 0x01 && m_usdio_message_in[conn_handle].payload[1] <= 0x30 && m_dnd_mode_active == false)
            {
                // Prüfen, ob sich etwas ge?ndert hat
                if(m_settings.alarm.armed != m_usdio_message_in[conn_handle].payload[0]
                    || (m_settings.alarm.silent && m_usdio_message_in[conn_handle].payload[1] != ALARM_MODE_SILENT)
                    || (m_settings.alarm.prealarm && m_usdio_message_in[conn_handle].payload[1] != ALARM_MODE_PREALARM)
                    || (!m_settings.alarm.prealarm && !m_settings.alarm.silent && m_usdio_message_in[conn_handle].payload[1] != 0)
                    || m_settings.alarm.threshold != m_usdio_message_in[conn_handle].payload[2])
                {
                    m_settings.alarm.armed = m_usdio_message_in[conn_handle].payload[0];
                    
                    m_settings.alarm.prealarm = (m_usdio_message_in[conn_handle].payload[1] == ALARM_MODE_PREALARM);
                    
                    if(m_settings.alarm.prealarm == false)
                        m_settings.alarm.silent = (m_usdio_message_in[conn_handle].payload[1] == ALARM_MODE_SILENT);
                    else
                        m_settings.alarm.silent = false;
                    
                    m_settings.alarm.threshold = m_usdio_message_in[conn_handle].payload[2];
                    
                    m_update_settings = true;
                    
                    // Alarm beenden, wenn deaktiviert
                    if(m_settings.alarm.armed == false)
                    {
                        all_sounds_stop();
                        alarmcheck_stop();
                    }
                    else
                    {
                        // Alarmauswertung neustarten
                        alarmcheck_stop();
                        alarmcheck_start();
                    }
                }
            }
        }
        break; // case ALARM_SETTINGS_PRO
        
        case SOUND_SETTINGS:
        {
            LOG_DBG("SOUND_SETTINGS %d", m_usdio_message_in[conn_handle].payload[0]);
            
            // Toneinstellung setzen
            if (m_usdio_message_in[conn_handle].payload[0] <= 0x07 && m_dnd_mode_active == false)
            {
                // Prüfen, ob sich etwas geändert hat
                if(m_settings.sound.ble != m_usdio_message_in[conn_handle].payload[0])
                {
                    m_settings.sound.ble = m_usdio_message_in[conn_handle].payload[0];
                    
                    switch(m_settings.sound.ble)
                    {
                        case 0: 
                            m_settings.sound.mode = SOUND_CONF_CLOSE | SOUND_CONF_WARNING;
                        break;
                        
                        case 1: 
                            m_settings.sound.mode = SOUND_CONF_WARNING;
                        break;
                        
                        case 2: 
                            m_settings.sound.mode = SOUND_CONF_CLOSE;
                        break;
                         
                        case 3: 
                            m_settings.sound.mode = SOUND_CONF_OFF;
                        break;
                        
                        case 4: 
                            m_settings.sound.mode = SOUND_CONF_OPEN;
                        break;
                        
                        case 5: 
                            m_settings.sound.mode = SOUND_CONF_OPEN | SOUND_CONF_CLOSE;
                        break;
                        
                        case 6: 
                            m_settings.sound.mode = SOUND_CONF_OPEN | SOUND_CONF_CLOSE | SOUND_CONF_WARNING;
                        break;
                        
                        case 7: 
                            m_settings.sound.mode = SOUND_CONF_OPEN | SOUND_CONF_WARNING;
                        break;
                    }
                    
                    LOG_DBG("m_settings.sound.mode = %d", m_settings.sound.mode);
            
                    m_update_settings = true;
                }
            }
            else if(m_dnd_mode_active)
            {
                LOG_DBG("Fehler - DND aktiv");
                // Fehlercode an App senden
                send_status(conn_handle, STATUS_ERR_DND_ACTIVE);
            }
            else
            {
                LOG_DBG("Fehler");
                // Fehlercode an App senden
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
            }
        }
        break; // case SOUND_SETTINGS
        
        case AUTO_OPEN_SETTINGS:
        {
            LOG_DBG("AUTO_OPEN %d - Threshold %d - AUTO_CLOSE %d - Threshold %d", m_usdio_message_in[conn_handle].payload[0], m_usdio_message_in[conn_handle].payload[1], m_usdio_message_in[conn_handle].payload[2], m_usdio_message_in[conn_handle].payload[3]);
            
            // Payload: 
            // [0] = Auto-Open an/aus (0/1)
            // [1] = Schwellwert Öffnen
            // [2] = Schließen an/aus (0/1)
            // [3] = Schwellwert Schließen
            
            if(m_dnd_mode_active)
            {
                LOG_DBG("Fehler - DND aktiv");
                // Fehlercode an App senden
                send_status(conn_handle, STATUS_ERR_DND_ACTIVE);
            }
            else if(m_usdio_message_in[conn_handle].payload[0] <= 1 && m_usdio_message_in[conn_handle].payload[2] <= 1)
            {
                // Prüfen, ob sich etwas geändert hat
                if(m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].open_active != m_usdio_message_in[conn_handle].payload[0]
                    || m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_open != m_usdio_message_in[conn_handle].payload[1]
                    || m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].close_active != m_usdio_message_in[conn_handle].payload[2]
                    || m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_close != m_usdio_message_in[conn_handle].payload[3])
                {
                    m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].open_active =  m_usdio_message_in[conn_handle].payload[0];
                    m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_open = m_usdio_message_in[conn_handle].payload[1];
                    m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].close_active = m_usdio_message_in[conn_handle].payload[2];
                    m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_close = m_usdio_message_in[conn_handle].payload[3];
                    
                    m_update_settings = true;
                }
            }
        }
        break; // case AUTO_OPEN_SETTINGS
        
        case SHARE_CODE:
        {
            LOG_DBG("SHARE_CODE %.2X-%.2X-%.2X", m_usdio_message_in[conn_handle].payload[0], m_usdio_message_in[conn_handle].payload[1], m_usdio_message_in[conn_handle].payload[2]);
            
            if(m_usdio_message_in[conn_handle].payload[0] <= 33 && m_usdio_message_in[conn_handle].payload[1] <= 33 && m_usdio_message_in[conn_handle].payload[2] <= 33)
            {
                // Prüfen, ob sich etwas geändert hat
                if(memcmp(m_settings.sharecode_flash, m_usdio_message_in[conn_handle].payload, 3) != 0)
                {
                    memcpy(m_settings.sharecode_flash, m_usdio_message_in[conn_handle].payload, 3);
                    
                    // Farbcode für Eingabe festlegen
                    set_colorcodes();
                    
                    // Im Flash speichern
                    m_update_settings = true;
                }
            }
            else if(m_usdio_message_in[conn_handle].payload[0] == 0xFF && m_usdio_message_in[conn_handle].payload[0] == 0xFF && m_usdio_message_in[conn_handle].payload[0] == 0xFF)
            {
                // Prüfen, ob sich etwas geändert hat
                if(m_settings.sharecode_flash[0] != 0x63 && m_settings.sharecode_flash[1] != 0x63 && m_settings.sharecode_flash[2] != 0x63)
                {
                    // Sharing-Code deaktivieren
                    memset(m_sharecode, 0xFF, 6);
                    memset(m_settings.sharecode_flash, 0x63, 3);
                    
                    // Im Flash speichern
                    m_update_settings = true;
                }
            }
            else
            {
                LOG_DBG("Fehler");
                // Fehlercode an App senden
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
            }
        }
        break; // case SHARE_CODE
        
        case GPS_SIGNAL:
        {           
            // Find-My Pairing aktivieren
            // if(!m_fmna_paired)
            // {
            //     uint32_t err = fmna_pairing_mode_enter();
            //     if (err) {
            //         LOG_DBG("Cannot enter the FMN pairing mode (err: %d)", err);
            //     } else {
            //         LOG_DBG("%s the FMN pairing mode", m_fmna_pairing_mode_active ? "Extending" : "Enabling");
            //          m_fmna_pairing_mode_active = true;
            //     }
            // }
            // else
            // {
                if(m_prealarm_active)
                {
                    k_timer_start(&m_reset_prealarm_timer, TEN_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
                }
                
                //Alarm deaktivieren
                all_sounds_stop();
                alarmcheck_stop();
                
                // Signaltonwiedergabe starten
                signalsound_start();
//            }
        }
        break; // case GPS_SIGNAL
        
        case DELETE_PEER:
        {
            LOG_DBG("DELETE_PEER");
            // Peer-Daten löschen
            if(m_usdio_message_in[conn_handle].payload[0] < MAX_PEER_COUNT && m_current_locking_state_chain == STATUS_MOTOR_2_OPENED && m_current_locking_state == STATUS_MOTOR_1_OPENED)
            {
                if(m_num_app_bonds == 1 && m_num_fob_bonds == 0)
                    m_start_factory_reset = true;
                else
                    delete_peer_data(m_usdio_message_in[conn_handle].payload[0]);
            }
            else
            {
                // Status zurückgeben
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
            }
        }
        break; // case DELETE_PEER
        
        case DND_MODE:
        {
            LOG_DBG("DND_MODE");
            if(m_usdio_message_in[conn_handle].payload[0] == 0 && m_dnd_mode_active == true)
            {
                // Nicht-Stören Modus deaktivieren
                m_dnd_mode_active = false;
                LOG_DBG("Nicht-Stören Modus deaktiviert");
                
                // Falls nötig, Alarmauswertung starten
                alarmcheck_start();
                
                // Erfolg melden
                send_status(conn_handle, STATUS_SUCCESS);
            }
            else if(m_usdio_message_in[conn_handle].payload[0] == 1 && m_dnd_mode_active == false)
            {
                m_dnd_mode_active = true;
                LOG_DBG("Nicht-Stören Modus aktiviert");
                
                // Erfolg melden
                send_status(conn_handle, STATUS_SUCCESS);
            }
            else
            {
                LOG_DBG("Fehler");
                // Fehlercode an App senden
                send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
            }
        }
        break; // DND_MODE

        case START_DFU:
        {
            // nrf_dfu_peer_data_t dfu_peer;
            // memset(&dfu_peer, 0, sizeof(nrf_dfu_peer_data_t));
            
            // // LTK aufteilen und in zwei Arrays speichern
            // memcpy(dfu_peer.ble_id.id_info.irk, m_connected_peer[conn_handle]->ltk, LTK_LEN / 2);
            // memcpy(dfu_peer.enc_key.enc_info.ltk, &m_connected_peer[conn_handle]->ltk[LTK_LEN / 2], LTK_LEN / 2);
            // // Auth-ID und HW-ID kopieren
            // dfu_peer.enc_key.master_id.ediv = m_connected_peer[conn_handle]->auth_id;
            // memcpy(dfu_peer.sys_serv_attr, m_connected_peer[conn_handle]->hw_id, HW_ID_LEN);
            // // CRC berechnen
            // uint32_t crc = crc32_compute((uint8_t*)&dfu_peer + 4, sizeof(nrf_dfu_peer_data_t) - 4, NULL);
            // dfu_peer.crc = crc; 
            // nrf_dfu_settings_peer_data_write(&dfu_peer);
            
        }
        break;

        default: 
        {
            // Befehl nicht erkannt
            // Fehlercode an App senden
            send_status(conn_handle, STATUS_ERR_NOT_ALLOWED);
        }
        break;
    }
}


void usdio_send_data(uint16_t conn_handle, ili_usdio_message_t* message_out)
{
//    LOG_DBG("usdio_send_data - Command %d to device %d", message_out->command, conn_handle);
    
    // Nichts senden, wenn kein Geräte verbunden ist
    if(m_peripheral_conn_count == 0 || m_connected_peer[conn_handle].used == false)
		return;
    
    if((m_auth_state[conn_handle] != AUTHORISED && message_out->command != CHALLENGE && message_out->command != LTK)  || conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		send_status(conn_handle, STATUS_ERR_NO_AUTH);
		LOG_DBG("Nicht autorisiert");
		return;
	}
    
    switch(message_out->command)
    {
        case CHALLENGE:
        {
            psa_generate_random(m_nonce[conn_handle], SHA256_LENGTH);
            
            // Befehl erstellen
            memcpy(message_out->payload, m_nonce[conn_handle], SHA256_LENGTH);
            message_out->payload_len = SHA256_LENGTH;
        }
        break;
        
        case LTK:
		{
            // muss hier stehen bleiben, sonst funktioniert das Anlernen nicht
        }
        break;
        
        case CLICK_CODE:
		{
			memcpy(message_out->payload, m_settings.colorcode_flash, 3);
            message_out->payload_len = 3;
		}
		break; // case CLICK_CODE   
        
        case LOCK_CONFIG_PRO:
		{
            message_out->payload_len = 27;
            // Schließzustand Bügel
            message_out->payload[0] = m_current_locking_state;                  // 1 Byte
            // Schließzustand Kette
            message_out->payload[1] = m_current_locking_state_chain;            // 1 Byte
            // Aktiver Alarm
            message_out->payload[2] = m_alarmsound_active;                      // 1 Byte
            // Firmware-Version Major
            message_out->payload[3] = FIRMWARE_MAJOR;                           // 1 Byte
            // Firmware-Version Minor
            message_out->payload[4] = FIRMWARE_MINOR;                           // 1 Byte
            // Hardware-Version
            message_out->payload[5] = m_uicr_data.hardware_version;             // 1 Byte
            // Bootloader Version
            message_out->payload[6] = m_bl_version;                             // 1 Byte
            // I LOCK IT Sub-Variante
            message_out->payload[7] = m_uicr_data.subvariant;                   // 1 Byte
            // Alarm-Einstellungen
            message_out->payload[8] = m_settings.alarm.armed;                   // 1 Byte
            // Alarm-Schwellwert
            message_out->payload[9] = m_settings.alarm.ble;                     // 1 Byte
            // Automatik-Einstellungen
            message_out->payload[10] = m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].open_active;      // 1 Byte
            message_out->payload[11] = m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].close_active;     // 1 Byte
            message_out->payload[12] = m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_open;   // 1 Byte
            message_out->payload[13] = m_settings.auto_lock[m_connected_peer[conn_handle].peer_data->auth_id].threshold_close;  // 1 Byte
            // Toneinstellungen
            message_out->payload[14] = m_settings.sound.ble;                    // 1 Byte
            // Batteriestand
            message_out->payload[15] = ili_battery_get_value(); // 1 Byte
            // Farbcode
            memcpy(&message_out->payload[16], m_settings.colorcode_flash, 3);   // 3 Byte
            // Sharing-Code
            memcpy(&message_out->payload[19], m_settings.sharecode_flash, 3);   // 3 Byte
            // Nicht-Stören Modus
            message_out->payload[22] = m_dnd_mode_active;                       // 1 Byte
            // Handsender+ verbunden
            message_out->payload[23] = (m_fob_conn != NULL);                    // 1 Byte
            // Einsteckkette erkannt
            message_out->payload[24] = m_chain_is_present;                      // 1 Byte
            // Applikations-Flag
            message_out->payload[25] = APPLICATION_IS_ACTIVE;                   // 1 Byte
            // Testergebnis Beschleunigungssensor
            message_out->payload[26] = m_acc_check_passed;                      // 1 Byte
            // erw. Diebstahlschutz
            message_out->payload[27] = 0;                   // 1 Byte
		}
		break; // case LOCK_CONFIG
        
        case LOCK_STATE:
		{
            message_out->payload_len = 2;
            message_out->payload[0] = m_current_locking_state;
            message_out->payload[1] = ili_battery_get_value();  // 1 Byte
		}
		break; // case LOCK_STATE
        
        case BATT_LEVEL:
        {
            message_out->payload_len = 1;
            message_out->payload[0] = ili_battery_get_value();  // 1 Byte
        }break;
        
        case GPS_UUID:
		{
			message_out->payload_len = 8;
			memcpy(message_out->payload, &m_uicr_data.app_id_lsb, 4);
            memcpy(&message_out->payload[4], &m_uicr_data.app_id_msb, 4);

            LOG_HEXDUMP_DBG(message_out->payload, 8, "GPS_UUID Payload");
		}
		break;
        
        default:
        {
            LOG_DBG("Ungültiger Befehl");
            return;
        }
    }
    
    message_out->auth_id = m_connected_peer[conn_handle].peer_data->auth_id;
        
    ili_parse_usdio_msg_out(conn_handle, message_out, m_app_msg_buffer[conn_handle], &m_app_msg_buffer_length[conn_handle]);
    
    // Daten senden
	uint8_t index = 0;      
    uint8_t length = m_app_msg_buffer_length[conn_handle];
    
    while(length > 0 && m_connected_peer[conn_handle].used)
	{
		m_usdio_indication_ack = false;
		k_timer_start(&m_usdio_ind_ack_timer, IND_ACK_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
		uint16_t bytes_to_send = MIN(length, 20);
        
        // conn_handle muss hier um NRF_SDH_BLE_CENTRAL_LINK_COUNT erhöht werden, damit an das richtige Handle gesendet wird
        uint32_t err_code = ili_send_usdio_ind(m_connected_peer[conn_handle].conn, &m_app_msg_buffer[conn_handle][index], bytes_to_send);
						
        LOG_DBG("ble_ili_usdio_data_send = %d", err_code);
        if(err_code == APP_SUCCESS)
        {
            // Warten bis Acknowledge eingetroffen		
            while(m_usdio_indication_ack == false) 
            {
                k_sleep(K_MSEC(50));
            }
        }

		k_timer_stop(&m_usdio_ind_ack_timer);
        
		length -= bytes_to_send;
		index += bytes_to_send;
	}
}


/**
 * @brief Stoppen aller Töne
 */
void all_sounds_stop()
{
    LOG_DBG("all_sounds_stop");

    if(m_alarmsound_active || m_prealarm_active || m_signalsound_active)
    {
        if(m_settings.alarm.prealarm)
        {
            LOG_DBG("Start m_reset_prealarm_timer");
            k_timer_start(&m_reset_prealarm_timer, TEN_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
        }

        if((m_settings.alarm.silent == false && m_dnd_mode_active == false) || m_signalsound_active)
        {
            ili_piezo_stop();
        }
        
        if(m_alarmsound_active)
        {
            // Alarm-Deaktivierung an Apps senden
            send_status(BLE_CONN_HANDLE_ALL, STATUS_ALARM_OFF);
            // Alarm-Deaktivierung an Handsender senden
            m_fob_status = ILI_C_ALARM_OFF;
            k_work_submit(&work_fob_send_status);
            
            led_off();
        }
        
        // Alarm-Timer stoppen
        k_timer_stop(&m_alarm_timer);
        
        m_alarmsound_active = false;
        m_prealarm_active = false;
        m_signalsound_active = false;
    }
}


/**
 * @brief Starten des Hauptmotors
 */
static void set_motion_detection(bool enable)
{
    // Während der Erstellung eines Bondings soll das Schloss nicht genutzt werden können
    if(m_bonding_mode_active)
        return;
    
    LOG_DBG("set_motion_detection");

    if(enable && m_current_locking_state == STATUS_MOTOR_1_OPENED && ili_motorcontroller_get_state() == MOTOR_STOP)
    {
        uint8_t ret = ACC_NO_MOVEMENT;
     
        LOG_DBG("Alarm scharf schalten");

        // Bewegungsprüfung durchführen, wenn keine Kette vorhanden ist
        if(m_chain_is_present == false)
            ret = accelerometer_check(250);

        if(ret == ACC_NO_MOVEMENT)
        {
            LOG_DBG("ACC_NO_MOVEMENT");
            m_acc_check_passed = true;

            m_current_locking_state = STATUS_MOTOR_1_CLOSED;

            if(m_chain_is_present && m_current_locking_state_chain != STATUS_MOTOR_2_CLOSED)
            {
                m_current_locking_state_chain = STATUS_MOTOR_2_CLOSING;
                led_timed(LED_R, LED_LOCKING);
        
                // Motor starten
                ili_motorcontroller_start(false);

                // Status "Schließen" senden
                send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state_chain);
            }
            else
            {
                // Schließzustand im RAM ablegen
                k_work_submit(&work_retention_write);

                alarmcheck_start();
                //Aktivierungston und -LED abspielen
                beep_start(ILI_PIEZO_SOUND_ARMED);
                led_timed(LED_R, LED_STATIC);
                
                // Benachrichtigung senden
                send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state);
            }

            return;
        }
        // Wenn Bewegung erkannt wurde ist der Selbsttest nicht gültig
        // Es kann aber davon ausgegangen werden, dass der Sesor korrekt arbeitet
        // bzw. wenn durch einen Defekt dauerhaft Bewegung gemeldet wird, wird das Abschließen auch verhindert
        else if((ret & ACC_MOVEMENT) != 0)
        {
            LOG_DBG("Bewegung erkannt beim Scharfschalten");
            send_status(BLE_CONN_HANDLE_ALL, STATUS_MOTOR_1_CLOSE_MOVED);
            
            m_fob_status = ILI_C_CLOSE_MOVED;
            k_work_submit(&work_fob_send_status);
            
            m_acc_check_passed = true;
        }
        // Wenn keine Bewegung erkannt wurde, aber der Selbsttest gescheitert ist
        // --> Defekter Sensor
        else //if((ret & ACC_MOVEMENT) == 0 && ((ret & ACC_FAILED_SELFTEST) != 0 || (ret & ACC_INVALID_PARAM) != 0))
        {
            LOG_DBG("ACC_FAILED_SELFTEST");
            m_acc_check_passed = false;
            
            // TODO: Anpassen, wenn das von den Apps unterstützt wird
            send_status(BLE_CONN_HANDLE_ALL, STATUS_MOTOR_1_CLOSE_MOVED);
            //send_status(BLE_CONN_HANDLE_ALL, STATUS_ERR_ACC_SELFTEST_FAILED);
        }
    }
    else if(enable == false && m_current_locking_state == STATUS_MOTOR_1_CLOSED && ili_motorcontroller_get_state() == MOTOR_STOP && m_alarmsound_active == false)
    {
        LOG_DBG("Alarm entschärfen");

        // Alarm und Alarmprüfung beenden
        all_sounds_stop();
        alarmcheck_stop();

        // Farbcode-Eingabe abbrechen
        if(m_colorcode_in_index > 0)
            abort_colorcode_input();

        if(m_charge_active == false)
        {
            m_play_batt_warning = true;

            // Batteriestand beim Oeffnen messen. Die Messung laeuft parallel zur
            // Motorfahrt, es wird also die Spannung unter Last erfasst.
            ili_battery_start();
        }

        m_current_locking_state = STATUS_MOTOR_1_OPENED;

        if(m_current_locking_state_chain != STATUS_MOTOR_2_OPENED && m_chain_is_present)
        {
            m_current_locking_state_chain = STATUS_MOTOR_2_OPENING;
            led_timed(LED_G, LED_UNLOCKING);

            // Motor starten
            ili_motorcontroller_start(true);

            // Status "Öffnen" oder "Schließen" senden
            send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state_chain);
        }
        else
        {
            // Schließzustand im RAM ablegen
            k_work_submit(&work_retention_write);
            // Deaktivierungston und -LED abspielen
            beep_start(ILI_PIEZO_SOUND_DISARMED);
            led_timed(LED_G, LED_STATIC);

            // Benachrichtigung senden
            send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state);

            if(m_service_code_state == SERVICE_CODE_CORRECT)
                m_start_factory_reset = true;
        }

        return;
    }
    
    // Im Fehlerfall eine Warnung ausgeben
    if(m_alarmsound_active == false)
    {
        //ili_led_strip_stop();
        // Rot blinken im Fehlerfall 
        led_timed(LED_R, LED_ERROR);
        
        beep_start(ILI_PIEZO_SOUND_WARNING);
        
        send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state);
        
        // Im Testmodus den nächsten Test ausführen
        if(m_test_active)
        {
            // Test weiterführen
            m_actual_test++;
            m_test_active = false;
        }
    }
}


/** @brief Starten der Alarmprüfung
*/
void alarmcheck_start()
{
    if(m_settings.alarm.armed && m_factory_condition == false
        && ili_motorcontroller_get_state() == MOTOR_STOP
        && m_alarmsound_active == false && m_prealarm_active == false 
        && m_current_locking_state == STATUS_MOTOR_1_CLOSED)
    {
        LOG_DBG("alarmcheck_start()");

        // Alarmzähler zurücksetzen
        m_alarmcounter = 0;
        m_prealarm_active = false;
        // Beschleunigungssensor aktivieren
        k_work_submit(&work_acc_sniff);
        
        // Pin-Interupt aktivieren
        gpio_add_callback(pin_accel_int1.port, &accel_int1_cb_data);
    }
}


/** @brief Stoppen der Alarmprüfung
*/
void alarmcheck_stop()
{
    LOG_DBG("alarmcheck_stop()");

    // Schließzustand im RAM ablegen
    k_work_submit(&work_retention_write);

    k_work_submit(&work_acc_sleep);
    gpio_remove_callback(pin_accel_int1.port, &accel_int1_cb_data);

    k_timer_stop(&m_check_alarm_timer);
}


int crypto_init(void)
{
	psa_status_t status;

	/* Initialize PSA Crypto */
	status = psa_crypto_init();
	if (status != PSA_SUCCESS)
		return APP_ERROR;

    uint8_t bytes[4];
    memcpy(bytes, &m_uicr_data.serial_number, 4);
    psa_hash_setup(&m_sha256, PSA_ALG_SHA_256);
    psa_hash_update(&m_sha256, bytes, 4);
    psa_hash_finish(&m_sha256, m_stk, sizeof(m_stk), &m_sha256_len);

	return APP_SUCCESS;
}


static bool is_pin_triggered(uint32_t pin_irq)
{
    if((m_triggered_pins & pin_irq) != 0)
    {
        m_triggered_pins ^= pin_irq;
        return true;
    }
    else
        return false;
}


static void in_pin_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    uint32_t bit_to_pin = 0;
    uint32_t now = 0;

	for (int i = 0; i < 32; i++) {
		if ((pins >> i) == 1) {
			bit_to_pin = i;
			break;
		}
	}
   // LOG_DBG("in_pin_handler - bit_to_pin %d", bit_to_pin);

    if(bit_to_pin == pin_charge.pin)
    {
        if(gpio_pin_get_dt(&pin_charge))
            m_triggered_pins |= IRQ_CHARGE_COMPLETED;
    }
    else if(bit_to_pin == pin_accel_int1.pin)
    {
        if(m_relock_state == RELOCK_MOVEMENT_CHECK)
        {
            m_triggered_pins |= IRQ_ACC_RELOCK;
        }
        else if(m_current_locking_state != STATUS_MOTOR_1_OPENED)
        {
            m_triggered_pins |= IRQ_ACC_ALARM;
        }
    }
}


static void motorcontroller_evt_handler(mc_evt_t evt)
{
    LOG_DBG("motorcontroller_evt_handler");

    switch(evt.type)
    {
        case MC_ENDPOS_REACHED:
        {
            LOG_DBG("MC_ENDPOS_REACHED");

            if(evt.last_state == MOTOR_OPEN)
                m_triggered_pins |= IRQ_MOTOR_2_OPENED;
            else if(evt.last_state == MOTOR_CLOSE)
                m_triggered_pins |= IRQ_MOTOR_2_CLOSED;
        }
        break;

        case MC_MOTOR_BLOCKED:
        case MC_ENDPOS_TIMEOUT:
        {
            LOG_DBG("MC_ENDPOS_TIMEOUT or MC_MOTOR_BLOCKED");
    
            m_current_locking_state_chain = STATUS_MOTOR_2_LOCK_STATE_UNKNOWN;
            k_work_submit(&work_retention_write);

            // Rote LED blinken lassen
            led_timed(LED_R, LED_ERROR);

            beep_start(ILI_PIEZO_SOUND_WARNING);

            if(evt.last_state == MOTOR_OPEN)
            {
                send_status(BLE_CONN_HANDLE_ALL, STATUS_MOTOR_2_OPEN_BLOCKED);
                m_fob_status = ILI_C_OPEN_BLOCKED;
            }
            else if(evt.last_state == MOTOR_CLOSE)
            {
                send_status(BLE_CONN_HANDLE_ALL, STATUS_MOTOR_2_CLOSE_BLOCKED);
                m_fob_status = ILI_C_CLOSE_BLOCKED;
            }

            alarmcheck_start();

            // Unbekannten Schließzustand senden, wenn Motor steht
            k_work_submit(&work_fob_send_status);
            k_msleep(100);
            send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state_chain);
            send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state);
        }
        break;

        default:
        break;
    }

    // Im Testmodus den nächsten Test ausführen
    if(m_test_active)
    {
        // Test weiterführen
        m_actual_test++;
        m_test_active = false;
    }
}


/** @brief Wird aufgerufen, sobald eine Batteriemessung abgeschlossen ist
 *  @param batt_level_changed true, wenn sich der Batteriestand geändert hat
*/
static void battery_finished_handler(bool batt_level_changed)
{
    LOG_DBG("battery_finished_handler - %d", ili_battery_get_value());

    if(ili_battery_get_value() < BATTERY_LOW)
    {
        // Batteriewarnung nur abspielen, wenn die Messung beim Öffnen ausgelöst wurde
        if(m_play_batt_warning)
        {
            beep_start(ILI_PIEZO_SOUND_LOW_BATT);
        }

        send_status(BLE_CONN_HANDLE_ALL, STATUS_BATT_LOW);
    }

    // Flag für Warnton zurücksetzen
    m_play_batt_warning = false;

    // Wenn sich der Batteriestand geändert hat, wird eine Notification an alle verbundenen Geräte gesendet
    // Die App muss dann den neuen Batteriestand anfragen
    if(batt_level_changed)
    {
        send_status(BLE_CONN_HANDLE_ALL, STATUS_BATT_CHANGED);
    }
}


static void plug_evt_handler(button_evt_t evt)
{
    switch(evt)
    {
        case CHAIN_BUTTON_EVT_PRESSED:
        {
            LOG_DBG("CHAIN_BUTTON_PRESSED");
            m_chain_is_present = true;
            if(ili_motorcontroller_get_state() == MOTOR_STOP)
                m_triggered_pins |= IRQ_PLUG_DETECTION;
        }
        break;

        case CHAIN_BUTTON_EVT_RELEASED:
        {
            LOG_DBG("CHAIN_BUTTON_RELEASED");
            m_chain_is_present = false;
            if(ili_motorcontroller_get_state() == MOTOR_STOP)
                m_triggered_pins |= IRQ_PLUG_DETECTION;
        }
        break;

        case MAIN_BUTTON_EVT_PRESSED:
        {
            LOG_DBG("MAIN_BUTTON_EVT_PRESSED");
        }
        break;

        case MAIN_BUTTON_EVT_RELEASED:
        {
            LOG_DBG("MAIN_BUTTON_EVT_RELEASED");
        }
        break;
        
        default:
        break;
    }
}

static void gpio_init()
{
    LOG_DBG("gpio_init()");
        
    //Initialisierung der Eingänge

    // Lade-Pin
    gpio_pin_configure_dt(&pin_charge, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_interrupt_configure_dt(&pin_charge, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&charge_cb_data, in_pin_handler, BIT(pin_charge.pin));
    gpio_add_callback(pin_charge.port, &charge_cb_data);

    // Interrupt Beschleunigungssensor
    gpio_pin_configure_dt(&pin_accel_int1, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&pin_accel_int1, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&accel_int1_cb_data, in_pin_handler, BIT(pin_accel_int1.pin));
    
    // GPS
    gpio_pin_configure_dt(&pin_gps_enable, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&pin_gps_button, GPIO_OUTPUT_INACTIVE);

    // Erkennung der Einsteckkette
    int ret = ili_button_init(plug_evt_handler);
    LOG_DBG("ili_button_init = %d", ret);
}


/**
 * @brief Bei Initialisierung den aktuellen Schließzustand ermitteln
 */
static void get_locking_state(void)
{
    // Die Variable für den Schließzustand wird in der zweiten RAM-Region abgelegt
    // Dadurch sollte sie bei einem Reset erhalten bleiben
    // Es kann dann hier bei einem Neustart überprüft werden, ob versucht wurde den Schließzustand
    // nach einem Neustart zu manipulieren
    if(retention_is_valid(m_ram))
    {
        retention_read(m_ram, 0, &m_current_locking_state_chain, 1);
        LOG_DBG("retention read");
    }
    
    LOG_DBG("m_current_locking_state_chain im RAM = %d", m_current_locking_state_chain);

    // Wenn in der Variable für den Schließzustand nichts plausibles steht
    // wird die Endlage neu abgefragt
    if(m_current_locking_state_chain != STATUS_MOTOR_2_OPENED
        && m_current_locking_state_chain != STATUS_MOTOR_2_CLOSED
        && m_current_locking_state_chain != STATUS_MOTOR_2_LOCK_STATE_UNKNOWN)
    {
        // Schließzustand setzen
        if(m_factory_condition)
            m_current_locking_state_chain = STATUS_MOTOR_2_OPENED;
        else
            m_current_locking_state_chain = STATUS_MOTOR_2_LOCK_STATE_UNKNOWN;
    }
    
    // Alarm-Zustand auslesen/festlegen
    if(m_factory_condition)
    {
        m_current_locking_state = STATUS_MOTOR_1_OPENED;
    }
    else if(retention_is_valid(m_ram))
    {
        retention_read(m_ram, 1, &m_current_locking_state, 1);
        LOG_DBG("retention read");
    }
    
    LOG_DBG("m_current_locking_state im RAM = %d", m_current_locking_state);

    // Kein korrekter Zustand gefunden
    if(m_current_locking_state != STATUS_MOTOR_1_CLOSED && m_current_locking_state != STATUS_MOTOR_1_OPENED)
    {
        if(m_current_locking_state_chain == STATUS_MOTOR_2_CLOSED)
        {
            m_current_locking_state = STATUS_MOTOR_1_CLOSED;
            LOG_DBG("m_current_locking_state = STATUS_MOTOR_1_CLOSED");
        }
        else
        {
            m_current_locking_state = STATUS_MOTOR_1_OPENED;
            
            LOG_DBG("m_current_locking_state = STATUS_MOTOR_1_OPENED");
        }
    }

    // Erkennen, ob Kette eingesteckt ist
    m_chain_is_present = (ili_button_get_chain_state() == CHAIN_BUTTON_EVT_PRESSED);
    LOG_DBG("Chain is %s", m_chain_is_present ? "present" : "not present");

    // Zustand auswerten und anzeigen
    if(m_chain_is_present)
    {
        // Endlage auswerten
        if(m_current_locking_state_chain == STATUS_MOTOR_2_CLOSED)
        {
            LOG_DBG("get_locking_state() - geschlossen - Alarm aktiv");
            // Rote LED anschalten
            led_timed(LED_R, LED_STATIC);

            m_current_locking_state = STATUS_MOTOR_1_CLOSED;
        }
        // Endlage Geöffnet erreicht
        else if(m_current_locking_state_chain == STATUS_MOTOR_2_OPENED)
        {
            LOG_DBG("get_locking_state() - geöffnet");
            if(m_current_locking_state == STATUS_MOTOR_1_CLOSED)
            {
                LOG_DBG("get_locking_state() - Alarm aktiv");
                // Rote LED anschalten
                led_timed(LED_R, LED_STATIC);

            }
            else
            {
                LOG_DBG("get_locking_state() - Alarm aus");
                // Grüne LED anschalten
                led_timed(LED_G, LED_STATIC);
            }
            
        }
        // Keine Endlage erreicht -> Fehlermeldung
        else
        {
            LOG_DBG("get_locking_state() - undefiniert");
            m_current_locking_state = STATUS_MOTOR_1_CLOSED;
            
            // rote LED blinken lassen  
            led_timed(LED_R, LED_ERROR);
        }
    }
    else
    {
        if(m_current_locking_state == STATUS_MOTOR_1_CLOSED)
        {
            // Rote LED anschalten
            led_timed(LED_R, LED_STATIC);
        }
        else
        {
            // Grüne LED anschalten
            led_timed(LED_G, LED_STATIC);
        }
    }
}


static int flash_init()
{
    int rc = 0;
	
    /* define the zms file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	4 sectors
	 *	starting at ZMS_PARTITION_OFFSET
	 */

    const struct flash_area *app_flash_area;
    rc = flash_area_open(FIXED_PARTITION_ID(ZMS_PARTITION), &app_flash_area);

    //LOG_DBG("app_flash_area->fa_size = 0x%.2X", app_flash_area->fa_size);
    //LOG_DBG("app_flash_area->fa_off = 0x%.2X", app_flash_area->fa_off);
    //LOG_DBG("app_flash_area->fa_id = 0x%.2X", app_flash_area->fa_id);
    
     //ZMS Struktur füllen 
    m_filesys.flash_device = app_flash_area->fa_dev;
    m_filesys.offset = app_flash_area->fa_off;
    m_filesys.sector_size = 4096;          // Beispiel: 4 KB Erase-Block
    m_filesys.sector_count = app_flash_area->fa_size / m_filesys.sector_size;

    // Mount
    rc = zms_mount(&m_filesys);
    if (rc) {
        LOG_DBG("zms_mount failed: %d\n", rc);
        return rc;
    }

    LOG_DBG("Flash successfully initialised");
    
    return 0;
}


/**
 * @brief Laden der gespeicherten Peer-Daten
 */
static void load_peer_data()
{	
	LOG_DBG("load_peer_data()");
	int read_bytes = 0;

	// Laden des aktuellen Speicherstands der Bondingdaten
	read_bytes = zms_read(&m_filesys, PEER_DATA_ID, &m_peer_data, sizeof(m_peer_data));
	if (read_bytes > 0) {
		LOG_DBG("Bondingdaten wurden gefunden");

// Benutzte Peer-Daten ausgeben
//			for(uint8_t i = 0; i < MAX_PEER_COUNT; i++)
//			{
//                LOG_RAW("%d:\r\n", i);
//				if(m_peer_data[i].is_used)
//				{						
//					LOG_RAW("Datensatz %d ausgeben:\r\nLTK: ", i);
//                    LOG_HEXDUMP_DBG(m_peer_data[i].ltk, LTK_LEN, "LTK");
//					LOG_RAW("Peer-HW-ID: ");
//					LOG_HEXDUMP_DBG(m_peer_data[i].hw_id, HW_ID_LEN, "HW-ID");
//					LOG_RAW("Peer-AUTH-ID: %d\r\n", m_peer_data[i].auth_id);
//				}
//			}

	}
}


/**
 * @brief Initialisieren der Einstellungen mit Standardwerten
 */
void init_settings()
{
    // Farbcodes initialisieren
    memset(m_settings.colorcode_flash, 99, 3);
    memset(m_settings.sharecode_flash, 99, 3);
    m_settings.alarm.armed = true;
    m_settings.alarm.threshold = ALARM_SENS_MID;
    m_settings.alarm.prealarm = true;
    m_settings.alarm.silent = false;
    m_settings.alarm.ble = ALARM_MODE_MID | ALARM_MODE_PREALARM;
    
    for(uint8_t i = 0; i < MAX_PEER_COUNT; i++)
    {
        m_settings.auto_lock[i].open_active = true;
        m_settings.auto_lock[i].close_active = false;
        m_settings.auto_lock[i].threshold_open = 40;
        m_settings.auto_lock[i].threshold_close = 60;
    }
    
    m_settings.sound.mode = SOUND_CONF_CLOSE | SOUND_CONF_WARNING | SOUND_CONF_OPEN;
    m_settings.sound.ble = 6;
    memset(m_settings.reserved, 0, 32);

    if(retention_is_valid(m_ram))
        retention_read(m_ram, RAM_BLVERSION_OFFSET, &m_bl_version, 1);    

    TD("Fix für iOS FW check");
    m_bl_version = 1;

    // Farbcode festlegen 6*Weiß
    memset(m_colorcode_service_code_enable, SELECTED_COLOR_INIT_VAL, 6);
}


/**
 * @brief Laden der Einstellungen
 */
void load_settings()
{
    int read_bytes = 0;

	LOG_DBG("Start searching Settings Data...");
	// Laden des aktuellen Speicherstands der Einstellungen
	read_bytes = zms_read(&m_filesys, SETTINGS_ID, &m_settings, sizeof(m_settings));
	if (read_bytes > 0) {
		LOG_DBG("Settings wurden gefunden");
        
        // Farbcode setzen
        set_colorcodes();
	} else   {
        LOG_DBG("Settings wurden nicht gefunden");
	}
}


static void wdt_init()
{
    if (!device_is_ready(m_watchdog)) {
		LOG_ERR("%s: device not ready.\n", m_watchdog->name);
		return;
	}

    struct wdt_timeout_cfg wdt_config = {
		/* Reset SoC when watchdog timer expires. */
		.flags = WDT_FLAG_RESET_SOC,

		/* Expire watchdog after max window */
		.window.min = WDT_MIN_WINDOW,
		.window.max = WDT_MAX_WINDOW,
	};

    m_wdt_channel_id = wdt_install_timeout(m_watchdog, &wdt_config);

	if (m_wdt_channel_id < 0) {
		LOG_ERR("Watchdog install error %d\n", m_wdt_channel_id);
		return;
	}

	int err = wdt_setup(m_watchdog, WDT_OPT);
	if (err < 0) {
		LOG_ERR("Watchdog setup error %d\n", err);
		return;
	}
}

//////////////////////////////////////////////////////////////
//                         UART                             //
//////////////////////////////////////////////////////////////
/**@brief Startet den UART-Empfang nach einem Fehler erneut.
 *
 * Wird aus dem UART_RX_DISABLED-Ereignis heraus verzögert eingeplant.
 */
static void uart_rx_restart_handler(struct k_work* work)
{
    if(m_gps_active == false || m_uart == NULL)
        return;

    m_uart_rx_buf_index = 0;

    int err = uart_rx_enable(m_uart, m_uart_rx_buf[m_uart_rx_buf_index],
                             GSM_RX_BUFF_SIZE, RECEIVE_TIMEOUT);
    if(err && err != -EBUSY)
        LOG_ERR("uart_rx_enable (restart) = %d", err);
}
K_WORK_DELAYABLE_DEFINE(work_uart_rx_restart, uart_rx_restart_handler);


/**@brief Funktion zum Aktivieren des GPS-Moduls.
 */
static void gps_on()
{
    if(m_uicr_data.variant >= VARIANT_GPS_2G && m_gps_active == false)
    {
        LOG_DBG("GPS an");
        
        gpio_pin_set_dt(&pin_gps_enable, 1);
        

        m_gps_active = true;
        
        // UART-Verbindung wird nur bei der 4G-Variante initialisiert
        if(m_uicr_data.variant == VARIANT_GPS_4G || (m_needed_tests & TEST_GSM) != 0)
        {
            /**
             * @brief Initialisierung des UART und Starten des Empfangs
             */

            if(m_uart == NULL)
            {
                LOG_ERR("GPS-UART nicht vorhanden - uart00 ist im DTS deaktiviert (CAN-SPI aktiv)");
                return;
            }

            if(!device_is_ready(m_uart))
            {
                LOG_ERR("%s: device not ready.", m_uart->name);
                return;
            }

            int err = uart_callback_set(m_uart, uart_evt_handler, NULL);
            if(err)
            {
                LOG_ERR("uart_callback_set = %d", err);
                return;
            }

            err = uart_rx_enable(m_uart, m_uart_rx_buf[m_uart_rx_buf_index], GSM_RX_BUFF_SIZE, RECEIVE_TIMEOUT);
            if(err)
                LOG_ERR("uart_rx_enable = %d", err);
        }
    }
}


/**@brief FFunktion zum Deaktivieren des GPS-Moduls.
 */
static void gps_off()
{
    if(m_gps_active && m_gps_test_active == false)
    {
        LOG_DBG("GPS aus");
        
        // UART-Verbindung wird nur bei der 4G-Variante initialisiert
        if(m_uicr_data.variant == VARIANT_GPS_4G || (m_needed_tests & TEST_GSM) != 0)
        {
            // Befehlsqueue leeren und Timer zum Senden stoppen
            k_timer_stop(&m_gsm_send_timer);
            // m_gsm_cmd_queue ist ein statisches Kernel-Objekt (K_MSGQ_DEFINE),
            // darf also nicht mit k_free() freigegeben werden.
            k_msgq_purge(&m_gsm_cmd_queue);
            m_gsm_send_cmd = false;
            
            gpio_pin_set_dt(&pin_gps_button, 0);

            // Muss vor uart_rx_disable() gesetzt werden, damit das
            // UART_RX_DISABLED-Ereignis den Empfang nicht neu startet.
            m_gps_active = false;
            k_work_cancel_delayable(&work_uart_rx_restart);

            if(m_uart != NULL)
            {
                uart_tx_abort(m_uart);    // laufenden Sendevorgang abbrechen
                uart_rx_disable(m_uart);  // Empfang beenden
            }
            LOG_DBG("UART uninit");
            
            // Initialisierung der UART-Pins um Energie zu sparen
            //nrf_gpio_cfg_output(PIN_GSM_TX);
            //nrf_gpio_pin_write(PIN_GSM_TX, 0);
            //nrf_gpio_input_disconnect(PIN_GSM_RX);
        }

        gpio_pin_set_dt(&pin_gps_enable, 0);
        m_gps_active = false;
        
        // Counter zum Deaktivieren nach Alarmfall im Low-Power Modus
        m_gps_lpw_counter = 0;
        // Timeout nur resetten, wenn es anderweitig wieder gesetzt wird
        m_gps_lpw_timeout = 0;
    }
}


static void evaluate_gsm_msg()
{
    LOG_DBG("evaluate_gsm_msg");
    
    bool send_ack = false;
    
    // Referenz-ID extrahieren
    memcpy(&m_gsm_ack_ref_id, &m_gsm_message_in.payload[m_gsm_message_in.payload_len - 4], 4);
    
    // Serverbefehl auslesen
    switch(m_gsm_message_in.command)
    {
        case CMD_STATUS_ONLINE:
        {
            LOG_DBG("CMD_ONLINE");
            
        }break;

        case CMD_THEFT_RESP:
        {
            LOG_DBG("CMD_THEFT_RESP");
            
            uint8_t command;

            if(k_msgq_peek(&m_gsm_cmd_queue, &command) == 0)
            {
                LOG_DBG("Command in FIFO: 0x%.2X", command);
                // Prüfen, ob Antwort auf Theft-Request bekommen
                if(command == CMD_THEFT_REQ)
                {
                    // Alten Befehl entfernen
                    k_msgq_get(&m_gsm_cmd_queue, &command, K_NO_WAIT);
                    LOG_DBG("Command aus FIFO entfernen");
                }
            }
        
            LOG_DBG("Payload Length = %d", m_gsm_message_in.payload_len);
            
            if(m_gsm_message_in.payload_len > 0)
            {
                LOG_DBG("Payload = %d", m_gsm_message_in.payload[0]);
                LOG_DBG("m_settings.theft_mode = %d", m_settings.theft_mode);
                
                // Gestohlen-Bit steht an zweiter Stelle, daher wird der Payload zum Vergleich um eine Stelle verschoben
                if((m_settings.theft_mode & THEFT_MODE_STOLEN) != (m_gsm_message_in.payload[0] << 1))
                {
                    if(m_gsm_message_in.payload[0])
                        m_settings.theft_mode |= THEFT_MODE_STOLEN;
                    else
                        m_settings.theft_mode ^= THEFT_MODE_STOLEN;
                    
                    LOG_DBG("Theft mode after setting = %d", m_settings.theft_mode);
                    m_update_settings = true;
                }
                    
                // Wenn nicht gestohlen und keine Befehle mehr zum Senden vorhanden
                // kann das GPS deaktiviert werden
                if((m_settings.theft_mode & THEFT_MODE_STOLEN) == 0 && 	k_msgq_num_used_get(&m_gsm_cmd_queue) == 0 && m_gps_lpw_timeout == 0)
                {
                    // Wenn nicht gestohlen, kann das GPS wieder ausgeschaltet werden
                    gps_off();
                }
            }
        }break;
        
        case CMD_SIGNAL_SOUND:
        {
            send_ack = true;
            
            if(m_prealarm_active)
            {
                k_timer_start(&m_reset_prealarm_timer, TEN_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
            }
            
            //Alarm deaktivieren
            all_sounds_stop();
            alarmcheck_stop();
            
            // Signaltonwiedergabe starten
            signalsound_start();
        }break;
        
        case CMD_ACK:
        {
            LOG_DBG("ACK received for 0x%.2X", m_gsm_message_in.payload[0]);
            uint8_t command;
            
            // Im Testmodus soll nur das erste ACK beachtet werden
            // Weitere ACKs werden übersprungen
            if(m_gsm_test_successfull)
            {
                return;
            }
            
            if(k_msgq_peek(&m_gsm_cmd_queue, &command) == 0)
            {
                LOG_DBG("Command in FIFO: 0x%.2X", command);
                // Prüfen, ob ACK für den richtigen Befehl erhalten
                if(command == m_gsm_message_in.payload[0])
                {
                    // Alten Befehl entfernen
                    k_msgq_get(&m_gsm_cmd_queue, &command, K_NO_WAIT);
                    
                    // Aktionen nach Erhalt eines ACKs ausführen
                    switch(command)
                    {
                        case CMD_TESTMODE_SERIAL_PRO:
                        {
                            if(!m_gsm_test_successfull)
                            {
                                gps_off();
                                k_msleep(100);
                                gsm_send(CMD_TESTMODE_SERIAL_PRO);
                            }
                            
                            // Wenn die Seriennummer übertragen wurde, war der Testmodus erfolgreich
                            m_gsm_test_successfull = true;
                            return;
                        }break;
                        
                        case CMD_STATUS_NOT_STOLEN:
                        {
                            // Nachdem das Schloss den erw. Diebstahlmodus verlassen hat
                            // kann auch der Timeout eines Alarms beendet und das GPS ausgeschaltet werden
                            m_gps_lpw_timeout = 0;
                        }break;
                    }
                    
                    // Anzahl der Elemente in der FIFO abfragen
                    if(k_msgq_num_used_get(&m_gsm_cmd_queue) > 0)
                    {
                        LOG_DBG("Weiteren Befehl in Queue versenden");
                        // Weitere Elemente in der Queue -> Nächsten Befehl senden
                        m_gsm_send_counter = 0;
                        m_gsm_send_cmd = true;
                    }
                    else
                    {
                        LOG_DBG("Queue leer - Timer stoppen");
                        // Keine weiteren Elemente vorhanden -> Timer beenden
                        k_timer_stop(&m_gsm_send_timer);
                        
                        LOG_DBG("m_gps_lpw_timeout: %d", m_gps_lpw_timeout);
                        LOG_DBG("m_gps_lpw_counter: %d", m_gps_lpw_counter);
                        
                        // Wenn das Modul bei der gesendeten Notification nicht aktiv bleiben soll
                        // und es nicht als gestohlen gemeldet ist, wird es abgeschaltet
                        if(m_gps_lpw_timeout == 0 && (m_settings.theft_mode & THEFT_MODE_STOLEN) == 0)
                        {
                            // GPS deaktivieren
                            gps_off();
                        }
                    }
                }
            }
            else
            {
                LOG_DBG("Nichts mehr in FIFO");
                // Keine Elemente vorhanden -> Timer beenden
                k_timer_stop(&m_gsm_send_timer);
            }
        }break;
        
        default:
        {
            gsm_send(CMD_ERR_WRONG_COMMAND);   
        }break;
    }
    
     // Acknowledgement senden und/oder Einstellungen speichern
    if(send_ack)
    {
        LOG_DBG("m_gsm_ack_ref_id = %d", m_gsm_ack_ref_id);
        uart_gsm_send_ack(m_gsm_message_in.command);
    }
}


void gsm_send(uint8_t command)
{
    // GPS anschalten, falls notwendig
    gps_on();
    
    // Daten werden nur bei der 4G-Variante versendet
    if(m_uicr_data.variant == VARIANT_GPS_4G || (m_needed_tests & TEST_GSM) != 0)
    {
        LOG_DBG("gsm_send() - 0x%.2X", command);
        uint8_t next_cmd;
        // Alarmmeldungen werden am Anfang der Liste eingefügt
        // Peek auf erstes Element in Liste -> Wenn Element vorhanden wird das gelesen und 0 zurückgeliefert
        if(k_msgq_peek(&m_gsm_cmd_queue, &next_cmd) == 0 && command == CMD_STATUS_ALARM)
        {
            // Erstes Element der Queue prüfen
            LOG_DBG("Next CMD = 0x%.2X", next_cmd);
            
            if(next_cmd != CMD_STATUS_ALARM)
            {
                LOG_DBG("Insert Alarm");
                // Alarmmeldung am Anfang der Queue einfügen
                k_msgq_put_front(&m_gsm_cmd_queue, &command);
                
                // Sende-Zähler zurücksetzen, 
                // da neuer Befehl an den Beginn der Liste gesetzt wurde
                m_gsm_send_counter = 0;
            }
            else
            {
                LOG_DBG("Skip Alarm");
            }
        }
        else
        {
            // Befehl in die Queue packen
            k_msgq_put(&m_gsm_cmd_queue, &command, K_NO_WAIT);
        }
        
        // FIFO war leer -> Befehl kann sofort gesendet und der Timer gestartet werden
        if (k_msgq_num_used_get(&m_gsm_cmd_queue) == 1)
        {
            LOG_DBG("FIFO leer - Senden");
            // Timer zur Kontrolle (neu)starten
            k_timer_start(&m_gsm_send_timer, GSM_SEND_TIMEOUT_INTERVAL, GSM_SEND_TIMEOUT_INTERVAL);
            
            // Datenpaket absenden
            m_gsm_send_counter = 0;
            m_gsm_send_cmd = true;
        }
    }
}


/**@brief   Funktion zum Senden eines Befehls über UART.
 *
 */
void uart_gsm_send(uint8_t command)
{
    LOG_DBG("uart_gsm_send");  

    static uint8_t gsm_msg_tx[UART_RX_BUF_SIZE];    
    
    if(m_gps_active == false)
    {
        LOG_DBG("GPS not active");
        return;
    }
    
    if(k_sem_take(&m_uart_tx_sem, K_NO_WAIT) != 0)
    {
        LOG_WRN("UART sendet noch");
        return;
    }

    gpio_pin_set_dt(&pin_gps_button, 1);
    // Pause für Aktivierung des UARTs    
    k_msleep(100);
    gpio_pin_set_dt(&pin_gps_button, 0);
    
    memset(gsm_msg_tx, 0, UART_TX_BUF_SIZE);
    
    // Header festlegen
    gsm_msg_tx[0] = 0x0A;
    gsm_msg_tx[1] = 0x0B;
    gsm_msg_tx[2] = command;
    
    switch(command)
    {
        // Schloss-Info
        case CMD_INFO_B2C:
        {
            LOG_DBG("CMD_INFO_B2C");
                        
            gsm_msg_tx[3]  = 16;
            gsm_msg_tx[4]  = FIRMWARE_MAJOR;
            gsm_msg_tx[5]  = 100;//ili_battery_get_value();
            gsm_msg_tx[6]  = m_current_locking_state;
            gsm_msg_tx[7]  = FIRMWARE_MINOR;
            gsm_msg_tx[8]  = (gpio_pin_get_dt(&pin_charge) == 0);
            gsm_msg_tx[9]  = m_settings.theft_mode;
            gsm_msg_tx[10] = m_settings.alarm.armed;
            gsm_msg_tx[11] = m_settings.alarm.ble;
            gsm_msg_tx[12] = m_settings.sound.ble;
            gsm_msg_tx[13] = m_bl_version;
            gsm_msg_tx[14] = m_current_locking_state_chain;
            // Platzhalter
            //gsm_msg_tx[15]
            //gsm_msg_tx[16]
            //gsm_msg_tx[17]
            //gsm_msg_tx[18]
            //gsm_msg_tx[19]
            
            break;
        }
        
        case CMD_TESTMODE_SERIAL_PRO:
        {
            LOG_DBG("CMD_TESTMODE_SERIAL_PRO");
            gsm_msg_tx[3] = ADV_NAME_LENGTH;
            memcpy(&gsm_msg_tx[4], m_uicr_data.advertising_name, ADV_NAME_LENGTH);
            
        }break;
        
        case CMD_STATUS_ALARM:
        {
            // Statusmeldung für Alarm (15 Min. online)
            LOG_DBG("Alarm-Meldung");
            gsm_msg_tx[3] = 0;
            m_gps_lpw_timeout = 3;
        }break;
        
        case CMD_THEFT_REQ:
        {
            // Abfrage des Diebstahlstatus alle 24h
            LOG_DBG("Diebstahl-Anfrage");
            gsm_msg_tx[3] = 0;
        }break;
        
        default:
        {
            // Statusmeldung (Alarm, Unfall, Schließfehler, Tracking-Modus etc.)
            LOG_DBG("default case for status and error messages");
            gsm_msg_tx[3] = 0;
            break;
        }
    }
    
    // CRC berechnen und an Nachricht anfügen
    uint16_t crc = crc16_itu_t(CRC_SEED, &gsm_msg_tx[2], gsm_msg_tx[3] + 2);
    memcpy(&gsm_msg_tx[gsm_msg_tx[3] + 4], &crc, 2);
    
    LOG_HEXDUMP_DBG(gsm_msg_tx, gsm_msg_tx[3] + 6, "Message: ");
    
    // Pause für Aktivierung des UARTs    
    k_msleep(100);
    int err = (m_uart != NULL)
              ? uart_tx(m_uart, gsm_msg_tx, gsm_msg_tx[3] + 6, UART_TX_TIMEOUT_US)
              : -ENODEV;
    if(err)
    {
        LOG_ERR("uart_tx = %d", err);
        k_sem_give(&m_uart_tx_sem);
    }

    // Counter für Sendeversuche außerhalb des Testmodus erhöhen
    if(m_needed_tests == TEST_NOT_FOUND)
        m_gsm_send_counter++;
}


void uart_gsm_send_ack(uint8_t command)
{
	LOG_DBG("uart_gsm_send_ack() - 0x%.2X", command);
	
    if(k_sem_take(&m_uart_tx_sem, K_NO_WAIT) != 0)
    {
        LOG_WRN("UART sendet noch");
        return;
    }

    // Header festlegen
    static uint8_t gsm_msg_tx[11] = {0x0A, 0x0B, 0xAC, 5, 0, 0, 0, 0, 0, 0, 0};
    
    if(m_gps_active)
    {
        gsm_msg_tx[4] = command;
        
        memcpy(&gsm_msg_tx[5], &m_gsm_ack_ref_id, 4);
        
        // CRC berechnen und an Nachricht anfügen
        uint16_t crc = crc16_itu_t(CRC_SEED, &gsm_msg_tx[2], gsm_msg_tx[3] + 2);
        memcpy(&gsm_msg_tx[9], &crc, 2);
        
        // Datenpaket absenden
        gpio_pin_set_dt(&pin_gps_button, 1);
        // Pause für Aktivierung des UARTs    
        k_msleep(100);
        gpio_pin_set_dt(&pin_gps_button, 0);
        k_msleep(100);
        
        int err = (m_uart != NULL)
                  ? uart_tx(m_uart, gsm_msg_tx, 11, UART_TX_TIMEOUT_US)
                  : -ENODEV;
        if(err)
        {
            LOG_ERR("uart_tx = %d", err);
            k_sem_give(&m_uart_tx_sem);
        }
    }
}
///////////////////////////////////////////////////////////////////////////////


/**
 * @brief Callback für die Ereignisse des UART-Treibers
 *
 * Wird im Interrupt-Kontext aufgerufen, daher hier keine langlaufenden
 * Aktionen ausführen.
 */
static void uart_evt_handler(const struct device* dev, struct uart_event* evt, void* user_data)
{
    switch(evt->type)
    {
        case UART_TX_DONE:
        case UART_TX_ABORTED:
            // Sendepuffer wieder freigeben
            k_sem_give(&m_uart_tx_sem);
            break;

        case UART_RX_RDY:
        {
            // Empfangene Bytes einzeln an den Parser weiterreichen
            const uint8_t* p_data = &evt->data.rx.buf[evt->data.rx.offset];

            for(size_t i = 0; i < evt->data.rx.len; i++)
            {
                uint32_t ret = ili_receive_gsm_msg(p_data[i], m_uart_rx_line, &m_uart_rx_line_len);

                if(ret == PARSER_MESSAGE_COMPLETE)
                {
                    // Nachricht sichern, damit sie in der Main-Schleife ausgewertet werden kann
                    if(m_gsm_msg_available == false && m_uart_rx_line_len <= sizeof(m_uart_rx_message))
                    {
                        memcpy(m_uart_rx_message, m_uart_rx_line, m_uart_rx_line_len);
                        m_uart_rx_message_len = m_uart_rx_line_len;
                        m_gsm_msg_available = true;
                    }
                    else
                    {
                        LOG_WRN("GSM-Nachricht verworfen (len = %d)", m_uart_rx_line_len);
                    }
                }
            }
        }
        break;

        case UART_RX_BUF_REQUEST:
        {
            // Dem Treiber die jeweils andere Hälfte des Doppelpuffers anbieten
            m_uart_rx_buf_index ^= 1;

            int err = uart_rx_buf_rsp(dev, m_uart_rx_buf[m_uart_rx_buf_index], GSM_RX_BUFF_SIZE);
            if(err)
            {
                // Puffer wurde nicht übernommen -> Index zurücksetzen
                m_uart_rx_buf_index ^= 1;
                LOG_WRN("uart_rx_buf_rsp = %d", err);
            }
        }
        break;

        case UART_RX_BUF_RELEASED:
            break;

        case UART_RX_DISABLED:
        {
            LOG_DBG("UART_RX_DISABLED");

            // Der Treiber schaltet den Empfang nach jedem Fehler ab.
            // Solange das GPS-Modul aktiv ist, muss der Empfang wieder
            // gestartet werden, sonst wird nie wieder etwas empfangen.
            // Der Neustart laeuft verzoegert ueber die Workqueue, damit ein
            // dauerhaft gestoerter RX-Pin keine Endlosschleife im ISR erzeugt.
            if(m_gps_active)
                k_work_schedule(&work_uart_rx_restart, K_MSEC(50));
        }
        break;

        case UART_RX_STOPPED:
            LOG_WRN("UART RX gestoppt (reason = %d)", evt->data.rx_stop.reason);
            break;

        default:
            break;
    }
}


/** @brief Aktion, die ausgeführt wird, wenn der Taster losgelassen wird
*/
void do_button_action()
{

}


static void evaluate_gpio_pins()
{
    if(is_pin_triggered(IRQ_USB_DETECTED))
    {
        LOG_DBG("IRQ_USB_DETECTED");
    }
    else if(is_pin_triggered(IRQ_CHARGE_COMPLETED))
    {
        LOG_DBG("IRQ_CHARGE_COMPLETED");

        // Wenn kein Nutzer verbunden, der Alarm deaktiviert, der Motor inaktiv, Farbcode-Eingabe nicht fehlerhaft, Find-My Pairing nicht aktiv
        // und der letzte Neustart länger als 30 Sek. her ist, wird ein Neustart durchgeführt
        if(authorised_user_connected() == false && m_alarmsound_active == false && ili_motorcontroller_get_state() == MOTOR_STOP && m_wrong_colorcode_attempts == 0
            && m_restart_allowed/* && m_fmna_pairing_mode_active == false && m_fmna_user_pairing_active == false*/ && m_bonding_mode_active == false)
        {
            m_restart_lock = true;
        }
        else
        {
            ili_battery_reset();
            ili_battery_start();
        }

        m_charge_active = false;

        if(!m_bonding_mode_active)
            led_off();
    }
    else if(is_pin_triggered(IRQ_USB_REMOVED))
    {
        LOG_DBG("IRQ_USB_REMOVED");
    }
    else if(is_pin_triggered(IRQ_BUTTON))
    {
        LOG_DBG("PIN_BUTTON with %d ticks", m_button_counter);
        
        if(m_bonding_mode_active == false)
            do_button_action();
        
    }
    else if(is_pin_triggered(IRQ_PLUG_DETECTION))
    {
        LOG_DBG("PIN_PLUG_DETECTION - lock_state = %d", m_current_locking_state);

        if(m_chain_is_present)
        {
            send_status(BLE_CONN_HANDLE_ALL, STATUS_CHAIN_CONNECTED);

            if(m_factory_condition == false && m_current_locking_state_chain != STATUS_MOTOR_2_CLOSED && m_chain_temp_disabled == false)
            {
                if(m_current_locking_state == STATUS_MOTOR_1_CLOSED)
                {
                    alarmcheck_stop();
                    m_current_locking_state = STATUS_MOTOR_1_OPENED;
                }

                if(ili_motorcontroller_get_state() == MOTOR_STOP)
                {
                    // Kurze Pause vor Bewegungsprüfung
                    k_msleep(1000);
                    set_motion_detection(true);
                }
            }
        }
        else
        {
            if(m_current_locking_state_chain != STATUS_MOTOR_2_CLOSED)
                send_status(BLE_CONN_HANDLE_ALL, STATUS_CHAIN_REMOVED);
    
            m_chain_temp_disabled = true;
            // Timer starten, um erneutes Verriegeln mit der Kette zu ermöglichen
            k_timer_start(&m_plug_reactivation_timer, PLUG_REACTIVATION_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
        }
    }
    else if(is_pin_triggered(IRQ_MOTOR_1_OPENED))
    {
        LOG_DBG("IRQ_MOTOR_1_OPENED");
    }
    else if(is_pin_triggered(IRQ_MOTOR_1_CLOSED))
    {
        LOG_DBG("IRQ_MOTOR_1_CLOSED");
    }
    else if(is_pin_triggered(IRQ_MOTOR_2_OPENED))
    {
        LOG_DBG("IRQ_MOTOR_OPENED");

        led_timed(LED_G, LED_STATIC);

        beep_start(ILI_PIEZO_SOUND_DISARMED);

        m_current_locking_state_chain = STATUS_MOTOR_2_OPENED;
        send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state_chain);
        send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state);

        // Schließzustand im RAM ablegen
        k_work_submit(&work_retention_write);

        if(m_service_code_state == SERVICE_CODE_CORRECT)
            m_start_factory_reset = true;

        // Wenn nur die Kette geöffnet und nicht der Alarm beendet wurde
        if(m_current_locking_state == STATUS_MOTOR_1_CLOSED)
        {
            // Einige Sekunden warten, bis der Alarm wieder aktiviert wird
            k_timer_start(&m_alarm_restart_timer, FIVE_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
        }

        LOG_DBG("Battery level: %d", ili_battery_get_value());
    }    
    else if(is_pin_triggered(IRQ_MOTOR_2_CLOSED))
    {
        LOG_DBG("IRQ_MOTOR_CLOSED");

        led_timed(LED_R, LED_STATIC);
        beep_start(ILI_PIEZO_SOUND_ARMED);

        // Schließzustand setzen und an App übertragen
        m_current_locking_state_chain = STATUS_MOTOR_2_CLOSED;
        send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state_chain);
        send_status(BLE_CONN_HANDLE_ALL, m_current_locking_state);

        // Schließzustand im RAM ablegen
        k_work_submit(&work_retention_write);

        m_current_locking_state = STATUS_MOTOR_1_CLOSED;
        alarmcheck_start();
    }
    else if(is_pin_triggered(IRQ_ACC_ALARM))
    {
        LOG_DBG("IRQ_ACC_ALARM");

        if(m_settings.alarm.armed && ili_motorcontroller_get_state() == MOTOR_STOP
            && m_signalsound_active == false)
        {
            if(m_alarmcounter == 0)
            {
                // Reset des Voralarms verzögern
                if(m_prealarm_fired)
                {
                    LOG_DBG("Restart m_reset_prealarm_timer");
                    k_timer_stop(&m_reset_prealarm_timer);
                    k_timer_start(&m_reset_prealarm_timer, TEN_SEC_TIMEOUT_INTERVAL, SINGLE_SHOT_TIMEOUT);
                }
                
                LOG_DBG("start m_check_alarm_timer");
                
                // Timer für das Stoppen der Alarmprüfung starten
                k_timer_start(&m_check_alarm_timer, CHECK_ALARM_TIMEOUT, SINGLE_SHOT_TIMEOUT);
            }

            m_alarmcounter++;

            // Anzahl der Alarmsamples hinzufügen, die während der Debounce-Zeit gesammelt wurden
            m_alarmcounter += m_collected_debounce_samples;
            m_collected_debounce_samples = 0;

            LOG_DBG("m_alarmcounter = %d", m_alarmcounter);

            // In Hauptschleife schieben
            if(m_alarmcounter > m_settings.alarm.threshold)
            {
                alarmcheck_stop();
                
                if(m_settings.alarm.prealarm && m_prealarm_fired == false && m_dnd_mode_active == false)
                    prealarm_start();
                else
                {
                    alarmsound_start();
                }
            }
            else
            {
                accelerometer_sniff();
            }
        }
    }
    else if(is_pin_triggered(IRQ_ACC_RELOCK))
    {
        
        
        if(m_relock_state == RELOCK_INACTIVE)
            return;
        
        LOG_DBG("IRQ_ACC_RELOCK");
    }
    else
    {
        LOG_DBG("UNKNOWN TRIGGERED EVENT - %d", m_triggered_pins);
        m_triggered_pins = 0;
    }
}


void factory_reset()
{
    int err;
    // Bondings mit Handsender löschen
    if(m_num_fob_bonds > 0)
    {
        err = bt_unpair(ILI_PRO_BT_ID, BT_ADDR_LE_ANY);
        if(err)
        {
            LOG_ERR("bt_unpair failed with %d", err);
            m_start_factory_reset = true;
        }
    }
    
    // Settings und Bondings im Flash löschen
    err = zms_clear(&m_filesys);
    if (err)
	{
		LOG_ERR("zms_clear failed!!");
        m_start_factory_reset = true;
	}
    
    // LTKs aus dem Secure Storage löschen
    for(uint8_t i = 0; i < MAX_PEER_COUNT; i++)
    {
        psa_destroy_key(ZEPHYR_PSA_APPLICATION_KEY_ID_RANGE_BEGIN + i);
    }

    // err = fmna_disable();
	// if (err) {
	// 	LOG_DBG("fmna_disable failed (err: %d)", err);
	// } else {
	// 	LOG_DBG("FMN disabled");
	// 	/* Reset the necessary flags. */
	// 	m_fmna_pairing_mode_active = false;
	// 	m_fmna_motion_detection_active = false;
	// }

    // Find-My Werkszustand herstellen
    // err = fmna_factory_reset();
	// if (err) {
	// 	LOG_DBG("fmna_factory_reset failed (err %d)", err);
	// }

    // Neustart triggern
    m_restart_lock = true;
}


void abort_colorcode_input()
{
    LOG_DBG("abort_colorcode_input");

    k_timer_stop(&m_colorcode_input_timer);
    k_timer_stop(&m_colorcode_timeout_timer);

    // eingegebenen Farbcode zurücksetzen
    m_colorcode_in_index = 0;
    m_selected_color = SELECTED_COLOR_INIT_VAL;
    memset(m_colorcode_in, 0, 6);
    // Farbcode-Eingabe beenden
    m_colorcode_input_active = false;

}

// static void fmna_sound_start(enum fmna_sound_trigger sound_trigger)
// {
//     m_fmna_sound_started = true;

// 	if (sound_trigger == FMNA_SOUND_TRIGGER_UT_DETECTION) {
// 		LOG_DBG("Play sound action triggered by the Unwanted Tracking Detection");

// 		beep_start(ILI_PIEZO_SOUND_FMNA_NON_OWNER);

// 	} else {
// 		LOG_DBG("Received a request from FMN to start playing sound from the connected peer");

// 		beep_start(ILI_PIEZO_SOUND_FMNA_OWNER);
// 	}

//     // Indication in 500 ms senden, dass der Ton beendet wurde
//     k_work_schedule(&work_fmna_sound_stopped, K_MSEC(1000));
    
// 	LOG_DBG("Starting to play sound...\n");
// }

// static void fmna_sound_stop(void)
// {
// 	LOG_DBG("Received a request from FMN to stop playing sound");
    
//     if(m_fmna_sound_started)
//     {
//         m_fmna_sound_started = false;
//         ili_piezo_stop();
//         fmna_sound_completed_indicate();
//     }
// }

// static const struct fmna_sound_cb fmna_sound_callbacks = {
// 	.sound_start = fmna_sound_start,
// 	.sound_stop = fmna_sound_stop,
// };


// static void fmna_motion_detection_start(void)
// {
// 	LOG_DBG("Starting motion detection...");

// 	m_fmna_motion_detection_active = true;

//     // Bewegungsmessung starten
//     k_work_submit(&work_fmna_motion_detect);
// }

// static bool fmna_motion_detection_period_expired(void)
// {
//     LOG_DBG("fmna_motion_detection_period_expired - %d", m_fmna_motion_detected);

//     // Neue Messung starten
//     k_work_submit(&work_fmna_motion_detect);

//     return m_fmna_motion_detected;
// }

// static void fmna_motion_detection_stop(void)
// {
// 	LOG_DBG("Stopping motion detection...");

// 	m_fmna_motion_detection_active = false;

//     // Letztes Bewegungssample zurücksetzen
//     memset(&m_last_fmna_sample, 0, sizeof(sample_t));
// }

// static const struct fmna_motion_detection_cb fmna_motion_detection_callbacks = {
// 	.motion_detection_start = fmna_motion_detection_start,
// 	.motion_detection_period_expired = fmna_motion_detection_period_expired,
// 	.motion_detection_stop = fmna_motion_detection_stop,
// };


// static void fmna_serial_number_lookup_exited(void)
// {
// 	LOG_DBG("Exited the FMN Serial Number lookup");
// }

// static const struct fmna_serial_number_lookup_cb fmna_sn_lookup_callbacks = {
// 	.exited = fmna_serial_number_lookup_exited,
// };


// static void fmna_location_availability_changed(bool available)
// {
// 	LOG_DBG("Find My location %s", available ? "enabled" : "disabled");

// 	m_fmna_location_available = available;

// 	neo_device_name_set(false);
// }


// static void fmna_battery_level_request(void)
// {
// 	LOG_DBG("Battery level request");
//     int err = fmna_battery_level_set(ili_battery_get_value());
// 	if (err) {
// 		LOG_DBG("fmna_battery_level_set failed (err %d)", err);
// 	}
// }

// static void fmna_pairing_failed(void)
// {
// 	LOG_DBG("FMN pairing has failed");
// }

// static void fmna_pairing_mode_exited(void)
// {
// 	LOG_DBG("Exited the FMN pairing mode");

// 	m_fmna_pairing_mode_active = false;
// }

// static void fmna_paired_state_changed(bool new_paired_state)
// {
// 	LOG_DBG("The FMN accessory transitioned to the %spaired state",
// 	       new_paired_state ? "" : "un");

// 	m_fmna_paired = new_paired_state;
// }


// static const struct fmna_info_cb fmna_info_callbacks = {
// 	.battery_level_request = fmna_battery_level_request,
// 	.pairing_failed = fmna_pairing_failed,
// 	.pairing_mode_exited = fmna_pairing_mode_exited,
// 	.paired_state_changed = fmna_paired_state_changed,
//     .location_availability_changed = fmna_location_availability_changed,
// };


// static int fmna_id_create(uint8_t id)
// {
// 	int ret;
// 	bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
// 	size_t count = ARRAY_SIZE(addrs);

// 	bt_id_get(addrs, &count);
// 	if (id < count) {
// 		return 0;
// 	}

// 	do {
// 		ret = bt_id_create(NULL, NULL);
// 		if (ret < 0) {
// 			return ret;
// 		}
// 	} while (ret != id);

// 	return 0;
// }


// static int fmna_initialize(void)
// {
// 	int err;

// 	err = fmna_sound_cb_register(&fmna_sound_callbacks);
// 	if (err) {
// 		LOG_DBG("fmna_sound_cb_register failed (err %d)", err);
// 		return err;
// 	}

// 	err = fmna_motion_detection_cb_register(&fmna_motion_detection_callbacks);
// 	if (err) {
// 		LOG_DBG("fmna_motion_detection_cb_register failed (err %d)", err);
// 		return err;
// 	}

// 	err = fmna_serial_number_lookup_cb_register(&fmna_sn_lookup_callbacks);
// 	if (err) {
// 		LOG_DBG("fmna_serial_number_lookup_cb_register failed (err %d)", err);
// 		return err;
// 	}

// 	err = fmna_id_create(FMNA_BT_ID);
// 	if (err) {
// 		LOG_DBG("fmna_id_create failed (err %d)", err);
// 		return err;
// 	}

// 	err = fmna_id_set(FMNA_BT_ID);
// 	if (err) {
// 		LOG_DBG("fmna_id_set failed (err %d)", err);
// 		return err;
// 	}

// 	err = fmna_battery_level_set(ili_battery_get_value());
// 	if (err) {
// 		LOG_DBG("fmna_battery_level_set failed (err %d)", err);
// 		return err;
// 	}

// 	err = fmna_info_cb_register(&fmna_info_callbacks);
// 	if (err) {
// 		LOG_DBG("fmna_info_cb_register failed (err %d)", err);
// 		return err;
// 	}

// 	err = fmna_enable();
// 	if (err) {
// 		LOG_DBG("fmna_enable failed (err %d)", err);
// 		return err;
// 	}

// 	return err;
// }


// static void identities_print(void)
// {
// 	char addr_str[BT_ADDR_LE_STR_LEN];
// 	bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
// 	size_t count = ARRAY_SIZE(addrs);

// 	bt_id_get(addrs, &count);

// 	if (count != CONFIG_BT_ID_MAX) {
// 		LOG_DBG("Wrong number of identities");
// 		k_oops();
// 	}

// 	bt_addr_le_to_str(&addrs[ILI_NEO_BT_ID], addr_str, sizeof(addr_str));
// 	LOG_DBG("ILI NEO identity %d: %s", ILI_NEO_BT_ID, addr_str);

// 	bt_addr_le_to_str(&addrs[FMNA_BT_ID], addr_str, sizeof(addr_str));
// 	LOG_DBG("Find My identity %d: %s", FMNA_BT_ID, addr_str);
// }


// static void neo_device_name_set(bool force)
// {
// 	static bool suffix_present = false;
// 	bool use_suffix;

// 	/* Suffix should be present when the HR sensor is in the pairing
// 	 * mode and when the Find My Network is enabled.
// 	 */
// 	use_suffix = (m_bonding_mode_active && m_fmna_location_available);

// 	if ((force) || (use_suffix != suffix_present)) {
// 		int err;
// 		const char* device_name = use_suffix ?
// 			m_uicr_data.advertising_name_fmna : m_uicr_data.advertising_name;

// 		err = bt_set_name(device_name);
// 		if (err) {
// 			LOG_DBG("bt_set_name failed (err %d)", err);
// 			return;
// 		}

// 		LOG_DBG("NEO device name set to: %s", device_name);

// 		suffix_present = use_suffix;

//         const struct bt_data adv_data[] = {
// 	        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//             BT_DATA(BT_DATA_NAME_COMPLETE, m_uicr_data.advertising_name, ADV_NAME_LENGTH)
//         };

// 		if (m_ili_neo_adv_pro) {
// 			err = bt_le_ext_adv_set_data(m_ili_neo_adv_pro,
// 						     adv_data, ARRAY_SIZE(adv_data),
// 						     NULL, 0);
// 			if (err) {
// 				LOG_DBG("bt_le_ext_adv_set_data failed (err %d)", err);
// 				return;
// 			}
// 		}
// 	}
// }


// static void fmna_activation_action_handle(void)
// {
// 	/* In case we are in error retry mode. */
// 	k_work_cancel_delayable(&work_fmna_enable);
// 	k_work_cancel_delayable(&work_fmna_disable);

// 	if (fmna_is_ready()) {
// 		fmna_disable_work_handle(NULL);
// 	} else {
// 		fmna_enable_work_handle(NULL);
// 	}
// }


enum mgmt_cb_return mcumgr_dfu_handler(uint32_t event, enum mgmt_cb_return prev_status,
                                int32_t *rc, uint16_t *group, bool *abort_more,
                                void *data, size_t data_size)
{
    // Prüfen, ob DFU erlaubt wird oder nicht
    switch(event)
    {
        case MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK:
        {
            LOG_DBG("MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK");
            // Nur ein autorisierter Nutzer verbunden, Batteriestand über 50% und geöffnet
            if(authorised_user_connected() && m_peripheral_conn_count == 1 && ili_battery_get_value() > 50 
                && m_current_locking_state_chain == STATUS_MOTOR_2_OPENED && m_current_locking_state == STATUS_MOTOR_1_OPENED)
            {
                LOG_DBG("DFU started");
                m_dfu_started = true;
            }
            else
            {
                LOG_DBG("Error");
                *abort_more = true;
                *rc = MGMT_ERR_EACCESSDENIED;
                return MGMT_CB_ERROR_ERR;
            }
        }break;

        case MGMT_EVT_OP_OS_MGMT_RESET:
        {
            LOG_DBG("MGMT_EVT_OP_OS_MGMT_RESET");
            if(!m_dfu_started)
            {
                *abort_more = true;
                *rc = MGMT_ERR_ENOTSUP;
                return MGMT_CB_ERROR_ERR;
            }
        }break;

        default:
        {
        }break;
    }
    
    return MGMT_CB_OK;
}


int main(void)
{
	int err;
    ssize_t bytes_written = 0;
    status_t status_msg;
    uint8_t gsm_command = 0;

	LOG_INF("===================== %d.%d =====================", FIRMWARE_MAJOR, FIRMWARE_MINOR);

//    power_configuration();

    // Registrieren der Callbacks für den MCU Manager
    mcumgr_dfu_cb.callback = mcumgr_dfu_handler;
    mcumgr_dfu_cb.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK;
    mgmt_callback_register(&mcumgr_dfu_cb);

    mcumgr_reset_cb.callback = mcumgr_dfu_handler;
    mcumgr_reset_cb.event_id = MGMT_EVT_OP_OS_MGMT_ALL;
    mgmt_callback_register(&mcumgr_reset_cb);

    // Watchdog initialisieren
    wdt_init();

    ili_piezo_init();

    ili_led_init();

    err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

    //LOG_INF("Bluetooth initialized\n");

    // Wird fürs Bonding benötigt
    if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_load();
	}

    init_settings();

    ili_read_uicr(&m_uicr_data);
    //LOG_INF("serial number = %d", m_uicr_data.serial_number);
    //LOG_INF("product type = %c", m_uicr_data.product_type);
    //LOG_INF("lock variant = %c", m_uicr_data.variant);
    //LOG_INF("lock subvariant = %c", m_uicr_data.subvariant);
    //LOG_INF("hardware version = %d", m_uicr_data.hardware_version);

    //LOG_RAW("Advertising-Name: ");
    //for(uint8_t i = 0; i < ADV_NAME_LENGTH; i++)
    //{
    //    LOG_RAW("%c", m_uicr_data.advertising_name[i]);
    //}
    //LOG_RAW("\n");
    
    LOG_HEXDUMP_INF(m_uicr_data.reset_code, 6, "Click-Code: ");

    //LOG_RAW("App-ID: 0x%.4X", m_uicr_data.app_id_msb);
    //LOG_RAW("%.4X\n", m_uicr_data.app_id_lsb);

	err = crypto_init();
	if (err) {
		LOG_ERR("Unable to initialise crypto library");
		return 0;
	}

    // RSSI Thread initialisieren
    rssi_init();

    // Timer initialisieren
    timers_init();

    // GPIOs initialisieren
    gpio_init();

    ili_motorcontroller_init(motorcontroller_evt_handler);

    // Handler für die Batteriemessung setzen
    ili_battery_set_handler(battery_finished_handler);
    
    //ili_led_strip_init();

    // Beschleunigungssensor initialisieren
    accelerometer_init(ILI_ACC_STK8321);

    // CAN-Controller initialisieren und sofort in den Standby legen
    ili_can_init();

    ble_services_init();

    advertising_init();

    err = flash_init();
    if (err) {
		LOG_ERR("Failed to init flash memory (err:%d)\n", err);
		return 0;
	}

    //get_locking_state();

    if(gpio_pin_get_dt(&pin_charge) == 0)
        m_charge_active = true;

    // Peer-Datensätze laden
	load_peer_data();

	m_num_app_bonds = get_bond_count();

    //err = bt_unpair(ILI_NEO_BT_ID, BT_ADDR_LE_ANY);
    scan_init();
    
    // err = fmna_initialize();
	// if (err) {
	// 	LOG_DBG("FMNA init failed (err %d)", err);
	// 	return err;
	// }

	// //LOG_DBG("FMNA initialized");

    // identities_print();

	// Falls Bondig existiert -> Eingerichtet Variable setzen
	if (m_num_fob_bonds > 0 || m_num_app_bonds > 0)
	{
		LOG_DBG("eingerichtet\tm_num_fob_bonds = %d\tm_num_app_bonds = %d", m_num_fob_bonds, m_num_app_bonds);
		m_factory_condition = false;

        // Einstellungen laden
        load_settings();
        
        advertising_start();

        // Scanning und Advertising starten
        if(m_num_fob_bonds > 0)
            scan_start();
            
        // Timer zur Messung einer Zeitspanne starten
        // Dient zum Starten einer Batteriemessung und zum Deaktivieren des GPS
        k_timer_start(&m_six_min_timer, SIX_MIN_TIMEOUT_INTERVAL, SIX_MIN_TIMEOUT_INTERVAL);
        
        // Alarm scharf schalten, wenn geschlossen
		alarmcheck_start();

        //m_start_factory_reset = true;
	}
	else
    {
		LOG_DBG("nicht eingerichtet");
        // Zur Sicherheit wird hier versucht die Settings zu löschen
        zms_delete(&m_filesys, SETTINGS_ID);
    }

    // Batteriestand messen
    //ili_battery_start();

    //ili_piezo_play(ILI_PIEZO_SOUND_ALARM);

     //advertising_start();

     //ili_motorcontroller_start(false);

TD("TODOs:");
// https://docs.nordicsemi.com/bundle/ncs-1.7.1/page/zephyr/guides/debug_tools/thread-analyzer.html
//https://docs.nordicsemi.com/bundle/ncs-2.5.2/page/zephyr/services/pm/device_runtime.html
// https://www.zephyrproject.org/power-play-optimizing-power-management-with-zephyr-videos/
    for (;;) {

        if(m_disconnect_device > 0)
        {
            for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
            {
                if((m_disconnect_device & (1 << i)) != 0)
                {
                    // Empfangsflag zurücksetzen
                    m_disconnect_device ^= (1 << i);
                    int err = bt_conn_disconnect(m_connected_peer[i].conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                    LOG_DBG("bt_conn_disconnect = %d", err);
                }
            }
        }

        if(m_stop_rssi)
        {
            m_stop_rssi = false;
            rssi_stop();
        }

        // Daten der Charaktieristiken auswerten
        if(m_gdio_data_received > 0)
        {
            for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
            {
                if((m_gdio_data_received & (1 << i)) != 0)
                {
                    // Empfangsflag zurücksetzen
                    m_gdio_data_received ^= (1 << i);
                    gdio_data_received(i);
                }
            }
        }
        else if(m_usdio_data_received > 0)
        {
            for(uint8_t i = 0; i < CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT; i++)
            {
                if((m_usdio_data_received & (1 << i)) != 0)
                {
                    // Empfangsflag zurücksetzen
                    m_usdio_data_received ^= (1 << i);
                    usdio_data_received(i);
                }
            }
        }

        // GPIOs auswerten
        if(m_triggered_pins > 0)
        {
            //LOG_DBG("m_triggered_pins = %d", m_triggered_pins);
            evaluate_gpio_pins();
        }

        // Empfangene UART-Nachricht auswerten
        if(m_gsm_msg_available)
        {
            LOG_HEXDUMP_DBG(m_uart_rx_message, m_uart_rx_message_len, "GSM RX: ");

            if(ili_parse_gsm_msg(m_uart_rx_message, m_uart_rx_message_len, &m_gsm_message_in) == PARSER_SUCCESS)
                evaluate_gsm_msg();
            else
                LOG_WRN("ili_parse_gsm_msg fehlgeschlagen");

            m_gsm_msg_available = false;
        }

        if(m_update_settings)
        {
            m_update_settings = false;
            
            TD("bytes written kann 0 sein, wenn sich nichts geändert hat. Sicherstellen, dass im Werkszustand keine Daten mehr vorhanden sind. Bzw. wenn 0 dann Settings löschen");

            bytes_written = zms_write(&m_filesys, SETTINGS_ID, &m_settings, sizeof(m_settings));

            LOG_DBG("bytes_written = %d", bytes_written);
            LOG_DBG("sizeof(m_settings) = %d", sizeof(m_settings));

            // Daten wurden erfolgreich geschrieben
            if(bytes_written == sizeof(m_settings))
            {
                // Bei Ersteinrichtung aus dem Werkszustand
                if(m_write_click_code)
                {
                    LOG_DBG("eingerichtet = true");
                    //Bonding war möglich, blaues Blinken beenden
                    k_timer_stop(&m_bond_timer);
                    m_bonding_mode_active = false;
                    // Blaue LED anschalten
                    led_timed(LED_B, LED_STATIC);
                        
                    // Timer zur Messung einer Zeitspanne starten
                    // Dient zum Starten einer Batteriemessung und zum Deaktivieren des GPS
                    k_timer_start(&m_six_min_timer, SIX_MIN_TIMEOUT_INTERVAL, SIX_MIN_TIMEOUT_INTERVAL);
                        
                    m_write_click_code = false;
                    // Am besten Anlernen nur für einen Peer gleichzeitig erlauben
                    if(m_peer_started_bonding != BLE_CONN_HANDLE_INVALID)
                    {
                        m_connected_peer[m_peer_started_bonding].used = true;
                        m_connected_peer[m_peer_started_bonding].peer_data->is_used = true;
                    }

                    m_update_peer_auth_id = m_connected_peer[m_peer_started_bonding].peer_data->auth_id;

                    LOG_DBG("m_update_peer_auth_id = %d", m_update_peer_auth_id);

                    // Find-My Pairing aktivieren
                    // if(!m_fmna_paired)
                    // {
                    //     err = fmna_pairing_mode_enter();
                    //     if (err) {
                    //         LOG_DBG("Cannot enter the FMN pairing mode (err: %d)", err);
                    //     } else {
                    //         LOG_DBG("%s the FMN pairing mode",
                    //             m_fmna_pairing_mode_active ? "Extending" : "Enabling");

                    //         m_fmna_pairing_mode_active = true;
                    //     }
                    // }
                }

                // Meldung über erfolgreiches Schreiben an App senden
                send_status(BLE_CONN_HANDLE_ALL, STATUS_SUCCESS);
            }
        }

        // Speichern von Bonding-Daten
		if(m_update_peer_auth_id != AUTH_ID_NONE)
		{
            LOG_DBG("m_update_peer_auth_id = %d", m_update_peer_auth_id);

            // Peer-ID für neues Bonding zurücksetzen
            m_peer_started_bonding = BLE_CONN_HANDLE_INVALID;

            // Sicherstellen, dass der Key vorher gelöscht wird, bevor er aktualisiert wird
            psa_destroy_key(ZEPHYR_PSA_APPLICATION_KEY_ID_RANGE_BEGIN + m_update_peer_auth_id);

            // Wenn die Peer-Daten nicht gelöscht wurden, dann wird der LTK im Secure Storage abgelegt
            // Ansonsten wird der Schritt übersprungen, da der LTK nicht mehr benötigt wird
            LOG_DBG("m_peer_data[m_update_peer_auth_id].is_used = %d", m_peer_data[m_update_peer_auth_id].is_used);

            if(m_peer_data[m_update_peer_auth_id].is_used)
            {
                // Key-Attribute festlegen
                mbedtls_svc_key_id_t key_id;
                psa_key_attributes_t key_attributes = PSA_KEY_ATTRIBUTES_INIT;
                psa_set_key_lifetime(&key_attributes, PSA_KEY_LIFETIME_PERSISTENT);
                psa_set_key_usage_flags(&key_attributes, PSA_KEY_USAGE_EXPORT);
                psa_set_key_type(&key_attributes, PSA_KEY_TYPE_AES);
                psa_set_key_bits(&key_attributes, LTK_LEN_BITS);
                
                // Key-ID festlegen
                // Anhand der Auth-ID können die Keys mit unterschiedlicher ID abgelegt werden
                psa_set_key_id(&key_attributes, ZEPHYR_PSA_APPLICATION_KEY_ID_RANGE_BEGIN + m_update_peer_auth_id);
                psa_status_t status = psa_import_key(&key_attributes, m_ltk, LTK_LEN, &key_id);
                LOG_DBG("psa_import_key = %d", status);
                LOG_DBG("import key id = 0x%.2X", key_id);
            }

            // Die restlichen Peer-Daten im normalen Flash ablegen
            (void)zms_write(&m_filesys, PEER_DATA_ID, &m_peer_data, sizeof(m_peer_data));

            m_update_peer_auth_id = AUTH_ID_NONE;

            // Anzahl der Peers aktualisieren
            m_num_app_bonds = get_bond_count();
            
            if(m_num_app_bonds > 0)
                m_factory_condition = false;
		}

   
        // Prüfen, ob eine Nachricht in der Queue liegt
        if (k_msgq_get(&m_status_msgq, &status_msg, K_NO_WAIT) == 0) {
            
            ili_gdio_message_t app_message_out;
            app_message_out.command = LOCK_STATUS;
            app_message_out.payload_len = 1;
            app_message_out.payload[0] = status_msg.status_code;
            
			gdio_send_data(status_msg.conn_handle, &app_message_out);	
        }
			
		

        if(m_start_factory_reset)
        {
            m_start_factory_reset = false;
            factory_reset();
        }

        if(m_restart_lock)
        {
            LOG_DBG("m_restart_lock = true");
            
            
            if(m_peripheral_conn_count > 0)
                send_status(BLE_CONN_HANDLE_ALL, STATUS_FACTORY_RESET_COMPLETE);
            else
            {
                // Wenn keine Geräte mehr verbunden sind, kann neugestartet werden
                LOG_DBG("SystemReset");
                k_msleep(1000);
                NVIC_SystemReset();
            }
        }

        if(m_gsm_send_cmd && ili_gsm_is_receiving() == false && m_update_settings == false)
        {
            m_gsm_send_cmd = false;
            
            err = k_msgq_peek(&m_gsm_cmd_queue, &gsm_command);
            if(err == 0)
            {
                uart_gsm_send(gsm_command);
            }
        }

        // Watchdog aktualisieren
        wdt_feed(m_watchdog, m_wdt_channel_id);

        // Pause in der Main-Schleife um anderen Threads
        // Zeit zum Ausführen zu geben 
        k_msleep(100);
	}
}
