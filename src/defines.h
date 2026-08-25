#ifndef DEFINES_H__
#define DEFINES_H__

// Firmware-Version
#define FIRMWARE_MAJOR  50
#define FIRMWARE_MINOR  53

#define CLOSE_BY_USER_AFTER_BLOCKING

// Memory Layout
// https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Flib_bootloader.html&anchor=lib_bootloader_memory
// https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v14.2.0%2Flib_bootloader_dfu_banks.html&cp=4_0_0_3_5_1_2_2&anchor=lib_bootloader_dfu_appdata

//Softdevice Memory Usage
// https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsds_s132%2FSDS%2Fs1xx%2Fmem_usage%2Fmem_usage.html

////////////////////////////////////////////////////////////////////////
////////////////////       Speicher-Konfiguration     //////////////////
////////////////////////////////////////////////////////////////////////

// Speicher nrf54L15 gesamt     0x17D000    1560576 Byte


// Verbleibender Platz f+r App  0xb5800     135168 Byte 	(festgelegt in pm_static.yml)

// Bank 0 ab Adresse            0x8800
// Bank 1 ab Adresse            0xbe000
// App-Speicher ab Addresse     0x174000    (Größe 0x4000)
// Settings ab Adresse          0x178000    (Größe 0x2000)
// Bootloader ab Adresse        0x00

////////// RAM ////////////
// RAM nrf54L15 gesamt          0x20000     131072 Byte
// RAM Start                    0x20000000 
// RAM Retention Adresse        0x2001ff00 (Größe 0x100)


///////// Connection Interval ///////
#define CONN_INT_MIN            0x0018
#define CONN_INT_MAX            0x003C

///////// RSSI-Auswertung ///////
#define RSSI_AVG_COUNT                      10
#define RSSI_STATE_FAR                      0x00
#define RSSI_STATE_NEAR                     0x01

//////////// RSSI-Einstellungen ////////////
#define DIST_VERY_VERY_NEAR                 6
#define DIST_VERY_NEAR                      5
#define DIST_NEAR                           1
#define DIST_MEAN                           2
#define DIST_FAR                            3
#define DIST_VERY_FAR                       4

//////////// Automatisches Schließen ////////////
#define RELOCK_INACTIVE                     0x00
#define RELOCK_AUTO_OPENED                  0x01
#define RELOCK_MOVEMENT_CHECK               0x02
#define RELOCK_CLOSED                       0x03

#define RELOCK_THRESHOLD                    30

//////////// Alarmeinstellungen ////////////
#define ALARM_SENS_LOWEST                   35
#define ALARM_SENS_LOW                      30
#define ALARM_SENS_MID                      25
#define ALARM_SENS_HIGH                     20
#define ALARM_SENS_HIGHER                   15
#define ALARM_SENS_HIGHEST                  10
#define ALARM_SENS_GPS_ACTIVE               20
#define ALARM_MODE_SILENT                   0x01
#define ALARM_MODE_LOWEST                   0x02
#define ALARM_MODE_LOW                      0x04
#define ALARM_MODE_MID                      0x08
#define ALARM_MODE_HIGH                     0x10
#define ALARM_MODE_PREALARM                 0x20
#define ALARM_MODE_HIGHER                   0x40
#define ALARM_MODE_HIGHEST                  0x80


///////// Toneinstellungen ///////
#define SOUND_CONF_OFF                      0x00
#define SOUND_CONF_WARNING                  0x01
#define SOUND_CONF_OPEN                     0x02
#define SOUND_CONF_CLOSE                    0x04

////////// PWM-Frequenzen //////////
#define FREQ_ALARM_MIN                      2900
#define FREQ_ALARM_MAX                      3500
#define FREQ_PREALARM_MIN                   3500
#define FREQ_PREALARM_MAX                   3500
#define FREQ_SIGNAL_MIN                     3000
#define FREQ_SIGNAL_MAX                     4000
#define FREQ_BEEP                           3500
#define FREQ_LOW_BATT_MIN                   5000
#define FREQ_LOW_BATT_MAX                   7500

///////// BLE-Timeouts ///////
#define BLE_NONE                            0x00
#define BLE_GDIO_ACK                        0x01
#define BLE_USDIO_ACK                       0x02
#define BLE_AUTH                            0x04

// Pin-Auswertung für Eingänge
#define IRQ_ACC_ALARM                       0x0001
#define IRQ_MOTOR_CLOSED                    0x0004
#define IRQ_MOTOR_OPENED                    0x0008
#define IRQ_DOUBLE_TAP_DETECTED             0x0010
#define IRQ_PLUG_DETECTION                  0x0020
#define IRQ_CHARGE_STARTED                  0x0040
#define IRQ_CHARGE_COMPLETED                0x0080
#define IRQ_ACC_COLORCODE                   0x0100
#define IRQ_SINGLE_TAP_DETECTED             0x0200

#define MAIN_BUTTON_PULL                    NRF_GPIO_PIN_PULLDOWN

// Startwert bei Farbcode-Eingabe
#define SELECTED_COLOR_INIT_VAL             3

// Zustand der Service-Code Eingabe
#define SERVICE_CODE_INACTIVE               0
#define SERVICE_CODE_ALLOWED                1
#define SERVICE_CODE_CORRECT                2

///////// LED-Blinkmodi ///////
#define LED_FLASH_ERROR                     0x01
#define LED_FLASH_DISCONNECT                0x02
#define LED_FLASH_BONDING                   0x03

///////// LED-Farben ///////
#define LED_OFF 0x00
#define LED_R   0x01
#define LED_G   0x02
#define LED_B   0x04
#define LED_W   0x07

///////// LED-PWM ///////
#define LED_PWM_SEQ_VALUE                   80

///////// Flash ///////
#define SETTINGS_ID         0x1111
#define SETTINGS_KEY        0x2222

#define PEER_DATA_ID        0x1112  // WICHTIG: Wenn die ID oder der KEY ge�ndert wird, muss dies auch im Bootloader ge�ndert werden
#define PEER_DATA_KEY       0x2223  // da sonst die Bonding-Daten nicht mehr ausgelesen werden k�nnen

// Flash Blocks für den Testmodus
#define TESTMODE_ID         0x3330
#define TESTMODE_KEY        0x4441

//////////// Verbindungsparameter ////////////
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL_MIN            510                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_INTERVAL_MAX            700                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< Unlimited advertising in general discoverable mode */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  K_MSEC(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   K_MSEC(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define SCAN_INTERVAL                       0x12C0
#define SCAN_WINDOW                         0x0050
#define SCAN_INTERVAL_BONDING               0x00C0
#define SCAN_WINDOW_BONDING                 0x00A0

#define MAX_FOB_BONDS                       3

//////////// UART-Konfiguration ////////////
#define GSM_TX_BUFF_SIZE 128
#define GSM_RX_BUFF_SIZE 10
#define RECEIVE_TIMEOUT 100

//////////// Error-Tag ////////////
#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//////////// Timer-Intervalle ////////////
#define SINGLE_SHOT_TIMEOUT                     K_MSEC(0)           // Makro für einen einmaligen Aufruf eines Timers 
#define BUTTON_DETECTION_INTERVAL               K_MSEC(50)          // Pause von einem GPIOTE event bis ein Taster als gedrückt erkannt wird
#define IND_ACK_TIMEOUT_INTERVAL                K_MSEC(2000)	    // Timeout für Indication Acknowledgement
#define BOND_LED_TIMEOUT_INTERVAL               K_MSEC(300)         // Intervall für das blaue Blinken beim Bonding
#define FDS_STORAGE_TIMEOUT_INTERVAL            K_MSEC(400)         // Intervall zum überprüfen der FDS-Aktivität
#define MOTOR_TIMEOUT_INTERVAL                	K_MSEC(15000) 	    // Motor abschalten
#define MC3630_TIMEOUT_INTERVAL                 K_MSEC(100)         // Intervall zum Auswerten der Bewegung
#define PREALARM_TIMEOUT_INTERVAL               K_MSEC(3500)        // Intervall bis zum Beenden des Voralarms
#define WATCHDOG_TIMEOUT_INTERVAL               K_MSEC(3000)        // Intervall fürs Nachladen des Watchdog Registers
#define BUTTONPRESS_TIMEOUT_INTERVAL            K_MSEC(50)          // Intervall für das Zählen der Klickdauer
#define BOND_TIMEOUT_INTERVAL                   K_MSEC(90000)       // Intervall für Bonding
#define PLUG_DETECTION_TIMEOUT_INTERVAL         K_MSEC(3000)        // Intervall zum Abschließen per Kettenerkennung
#define GSM_SEND_TIMEOUT_INTERVAL               K_MSEC(20000)       // Intervall zum Versenden eines GSM-Befehls
#define ACC_TEST_TIMEOUT_INTERVAL               K_MSEC(2000)        // Intervall bis zum Selbsttest des Beschleunigungssensors
#define CHECK_ALARM_TIMEOUT                     K_MSEC(4000)        // Intervall für Alarmprüfung
#define BEEP_TIMEOUT_INTERVAL                   K_MSEC(500)         // Intervall zum Beenden eines Pieptons
#define WARN_TIMEOUT_INTERVAL                   K_MSEC(200)         // Intervall zum Wiederholen des Warntons
#define PLUG_REACTIVATION_TIMEOUT_INTERVAL      K_MSEC(2000)        // Intervall zum Reaktivieren des Kettentasters nach Abzug der Kette
#define CHECK_AUTO_CLOSE_TIMEOUT                K_MSEC(30000)       // Intervall für die Bewegungsüberprüfung nach autom. Öffnen
#define RESET_WRONG_COLORCODE_TIMEOUT_INTERVAL  K_MSEC(180000)      // Intervall zum Prüfen der falschen Eingaben des Farbcodes
#define HALLSENSOR_TIMEOUT_INTERVAL             K_MSEC(3000)        // Timeout bis Hallsensor Interrupt aktiviert wird
#define ONE_SEC_TIMEOUT_INTERVAL                K_MSEC(1000)        // 1 Sek. Timeout
#define TWO_SEC_TIMEOUT_INTERVAL                K_MSEC(2000)        // 2 Sek. Timeout
#define THREE_SEC_TIMEOUT_INTERVAL              K_MSEC(3000)        // 3 Sek. Timeout
#define FIVE_SEC_TIMEOUT_INTERVAL               K_MSEC(5000)        // 5 Sek. Timeout
#define TEN_SEC_TIMEOUT_INTERVAL                K_MSEC(10000)       // 10 Sek. Timeout
#define TWENTY_SEC_TIMEOUT_INTERVAL             K_MSEC(20000)       // 20 Sek. Timeout
#define THIRTY_SEC_TIMEOUT_INTERVAL             K_MSEC(30000)       // 30 Sek. Timeout
#define ONE_MIN_TIMEOUT_INTERVAL                K_MSEC(60000)       // 60 Sek. Timeout
#define THREE_MIN_TIMEOUT_INTERVAL              K_MSEC(180000)      // 180 Sek. Timeout
#define SIX_MIN_TIMEOUT_INTERVAL                K_MSEC(360000)      // 6 Minuten Timeout zum Messen einer Zeitspanne
#define DOUBLE_TAP_TIMEOUT_INTERVAL             K_MSEC(500)         // Timeout zum Anzeigen des erkannten Doppeltaps


///////// Testmodi ///////
#define TEST_NONE                           0x00
#define TEST_MECHANIC                       0x0100
#define TEST_GSM                            0x0200
#define TEST_NOT_FOUND                      0x8000

///////// RAM-Offsets für Variablen ///////
#define RAM_LOCKSTATE_OFFSET                0
#define RAM_BLVERSION_OFFSET                10

///////// Stackgröße für RSSI-Thread ///////
#define RSSI_STACKSIZE 512

///////// Stackgröße für LED-Thread ///////
#define LED_STACKSIZE 512

///////// Watchdog Konfiguration ///////
#define WDT_MAX_WINDOW  5000U
#define WDT_MIN_WINDOW  0U
#define WDT_OPT WDT_OPT_PAUSE_IN_SLEEP// WDT_OPT_PAUSE_HALTED_BY_DBG gibt es auch noch

///////// Find-My Advertising-Konfiguration ///////
#define ILI_PRO_BT_ID   BT_ID_DEFAULT
#define FMNA_BT_ID      1

#endif // #ifndef DEFINES_H__
