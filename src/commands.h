
#ifndef COMMANDS_H__
#define COMMANDS_H__
 
// Befehle f�r BLE-Kommunikation
////////// Befehlsliste /////////
#define REQUEST_DATA                        0x0001
#define SERIALNUMBER                        0x0003
#define CHALLENGE                           0x0004
#define CHALLENGE_GPS                       0x0044
#define AUTH_AUTHENTICATOR                  0x0005
#define AUTH_DATA                           0x0006
#define AUTH_ID                             0x0007
#define AUTH_ID_CONF                        0x0008
#define IV                                  0x0009
#define LTK                                 0x000A
#define LTK_CONF                            0x000B
#define LOCK_STATUS                         0x000E
#define LOCK_CONFIG                         0x000F
#define LOCK_CONFIG_PRO                     0x000D
#define CHALLENGE_RESP                      0x0010
#define LOCK_ACTION                         0x0011
#define CLICK_CODE                          0x0012
#define ALARM_SETTINGS                      0x0013 // Alter Befehl f�r die Alarmeinstellungen
#define SOUND_SETTINGS                      0x0014
#define AUTO_SETTINGS                       0x0015 // Alter Befehl f�r den Automatikmodus 
#define SHARE_CODE                          0x0016
#define LOCK_STATE                          0x0017
#define DEVICE_SETTINGS                     0x0018
#define ALARM_TIMES                         0x0019
#define DELETE_PEER                         0x001A
#define BATT_LEVEL                          0x001B
#define START_DFU                           0x001C
#define DND_MODE                            0x001D
#define KEYFOB_INFO                         0x001E
#define BOND_INFO                           0x001F
#define SMARTPHONE_INFO                     0x0020
#define CLICK_CODE_TEST                     0x0021
#define DFU_CHALLENGE                       0x0022
#define GPS_UUID                            0x0023
#define GPS_SIGNAL                          0x0024
#define AUTO_OPEN_SETTINGS                  0x0025  // Befehl f�r den neuen Automatikmodus
#define ALARM_SETTINGS_PRO                  0x0026  // Befehl f�r die neue Alarmeinstellung

////////// STATUSCODES ////////////
#define STATUS_SUCCESS                      0x00
#define STATUS_MOTOR_1_CLOSING              0x01
#define	STATUS_MOTOR_1_CLOSED               0x02
#define STATUS_MOTOR_1_CLOSE_BLOCKED        0x03
#define STATUS_MOTOR_1_CLOSE_MOVED          0x04
#define STATUS_APP_MOVEMENT                 0x05
#define STATUS_MOTOR_1_OPENING              0x0A
#define STATUS_MOTOR_1_OPENED               0x0B
#define STATUS_MOTOR_1_OPEN_BLOCKED         0x0C
#define STATUS_MOTOR_1_LOCK_STATE_UNKNOWN   0x0F
#define STATUS_ALARM_ON                     0x10
#define STATUS_ALARM_OFF                    0x20
#define STATUS_BOND_COMPLETE                0x30
#define STATUS_CONN_COMPLETE                0x31
#define STATUS_BATT_LOW                     0x40
#define STATUS_BATT_CHANGED                 0x41
#define STATUS_MAX_BONDS                    0x50
#define STATUS_BOND_ERASED                  0x51
#define STATUS_BOND_NOT_FOUND               0x52
#define STATUS_FACTORY_RESET_COMPLETE       0x53
#define STATUS_BOND_FROM_HS                 0x60
#define STATUS_FOB_CONNECTED                0x70
#define STATUS_FOB_DISCONNECTED             0x80
#define STATUS_FOB_BONDINGMODE              0x81
#define STATUS_CHAIN_CONNECTED              0x82
#define STATUS_CHAIN_REMOVED                0x83
#define STATUS_ERR_GENERAL                  0x90
#define STATUS_ERR_NOT_ALLOWED              0x91
#define STATUS_ERR_CRC                      0x92
#define STATUS_ERR_TIMEOUT                  0x93
#define STATUS_ERR_DND_ACTIVE               0x94
#define STATUS_ERR_WRONG_MSG_SIZE           0x95
#define STATUS_ERR_WRONG_AUTH_ID            0x96
#define STATUS_ERR_AUTH_FAILED              0x97
#define STATUS_ERR_BOND_FAILED              0x98
#define STATUS_ERR_NO_AUTH                  0x99
#define STATUS_ERR_WRONG_COUNTER            0x9A
#define STATUS_ERR_ENTER_DFU_FAILED         0x9B
#define STATUS_ERR_DFU_TOO_MANY_CONNECTIONS 0x9C
#define STATUS_ERR_ACC_SELFTEST_FAILED      0x9D
#define STATUS_ERR_BONDING_ACTIVE           0x9E
#define STATUS_MOTOR_2_CLOSING              0xB1
#define	STATUS_MOTOR_2_CLOSED               0xB2
#define STATUS_MOTOR_2_CLOSE_BLOCKED        0xB3
#define STATUS_MOTOR_2_OPENING              0xBA
#define STATUS_MOTOR_2_OPENED               0xBB
#define STATUS_MOTOR_2_OPEN_BLOCKED         0xBC
#define	STATUS_MOTOR_2_LOCK_STATE_UNKNOWN   0xBF
#define STATUS_EMPTY                        0xFF

////////// Befehlsargumente ////////////
#define LOCK_ACTION_DISABLE_ALARM           0x01
#define LOCK_ACTION_ENABLE_ALARM            0x02
#define LOCK_ACTION_OPEN_CHAIN              0x03
#define LOCK_ACTION_CLOSE_CHAIN             0x04
#define LOCK_ACTION_CHECK_MOVEMENT          0x06
#define LOCK_ACTION_APP_MOVEMENT_NO         0x09
#define LOCK_ACTION_APP_MOVEMENT_YES        0x0A
#define DEVICE_SETTINGS_FACTORY_RESET       0x02
#define DEVICE_SETTINGS_NEW_BOND            0x05
#define DEVICE_SETTINGS_QUIT_ALARM          0x0A
#define DEVICE_SETTINGS_TEST_GPS            0x11

////////// App/Bootloader Flag ////////////
#define APPLICATION_IS_ACTIVE               0x0A
#define BOOTLOADER_IS_ACTIVE                0x0B

// Befehle f�r GSM-Kommunikation
#define GPS_RETRY_COUNT             6

#define CMD_STATUS                          0x01    // Statusmeldungen, Alarm, Ladevorgang beendet, Unfallerkennung, nicht geschlossen (Automatik), Fehler bei �ffnen/Schlie�en, etc...
#define CMD_INFO                            0x02    // FW-Version, Batteriestand, Schlie�zustand
#define CMD_THEFT_REQ                       0x03    // Diebstahlmeldung von Server an Schloss
#define CMD_SIGNAL_SOUND                    0x04    // Signalton abspielen
#define CMD_GPS_RESET                       0x05    
#define CMD_LOCKSTATE                       0x06    
#define CMD_LOCK_RESET                      0x07
#define CMD_LOCKING_REQ                     0x08
#define CMD_DEACTIVATE_ALARM                0x09
#define CMD_CAN_INFO                        0x0A
#define CMD_GET_CAN_INFO                    0x0B
#define CMD_GPS_OFF                         0x0C
#define CMD_BUTTON_OFF                      0x0D
#define CMD_BUTTON_ON                       0x0E
#define CMD_FOB_REQ                         0x10
#define CMD_FOB_RESP                        0x11
#define CMD_RFID_REQ                        0x1A
#define CMD_RFID_RESP                       0x1B
#define CMD_LOCKING_CHALLENGE               0x1C
#define CMD_LOCKING_RESP                    0x1D
#define CMD_THEFT_RESP                      0x1E
#define CMD_INFO_B2C                        0x1F

#define CMD_CONF_ALARM                      0x20
#define CMD_CONF_ALARM_MODE                 0x21
#define CMD_CONF_COLORCODE                  0x22
#define CMD_CONF_SOUND                      0x23
#define CMD_CONF_POSITION_PERIODIC          0x24
#define CMD_CONF_NOTIFICATIONS              0x26
#define CMD_CONF_NOTIFICATION_TRACKALL                  0xA0
#define CMD_CONF_NOTIFICATION_TRACKLPW                  0xA1
#define CMD_CONF_NOTIFICATION_TIMEOUTS                  0x27
#define CMD_CONF_AVAILABILITY                           0x28
#define CMD_CONF_AVAILABILITY_OPENED_BATT_LEVEL         0xA0
#define CMD_CONF_AVAILABILITY_OPENED_LPW_INTERVAL       0xA1
#define CMD_CONF_AVAILABILITY_CLOSED_BATT_LEVEL         0xA2
#define CMD_CONF_AVAILABILITY_CLOSED_LPW_INTERVAL       0xA3
#define CMD_CONF_AVAILABILITY_THEFT_BATT_LEVEL          0xA4
#define CMD_CONF_AVAILABILITY_THEFT_LPW_INTERVAL        0xA5
#define CMD_CONF_AVAILABILITY_THEFT_TRACKING_INTERVAL   0xA6
#define CMD_CONF_AVAILABILITY_TRACKING_INTERVAL         0xA7
#define CMD_CONF_EXTRAS                     0x29

#define CMD_STATUS_ALARM                    0x30
#define CMD_STATUS_LOAD_COMPLETE            0x31
#define CMD_STATUS_CLOSE_BLOCKED            0x32
#define CMD_STATUS_OPEN_BLOCKED             0x33
#define CMD_STATUS_LOCK_MOVED               0x34
#define CMD_STATUS_CRASH_DETECTION          0x35
#define CMD_STATUS_ONLINE                   0x36

#define CMD_STATUS_OPENED_CODE              0x40
#define CMD_STATUS_CLOSED_BUTTON            0x41
#define CMD_STATUS_OPENED_KEYFOB            0x42
#define CMD_STATUS_CLOSED_KEYFOB            0x43
#define CMD_STATUS_OPENED_RFID              0x44
#define CMD_STATUS_CLOSED_RFID              0x45
#define CMD_STATUS_OPENED_APP               0x46
#define CMD_STATUS_CLOSED_APP               0x47
#define CMD_STATUS_OPENED_GSM               0x48
#define CMD_STATUS_CLOSED_GSM               0x49
#define CMD_STATUS_TRACK_ALL                0x4A
#define CMD_STATUS_TRACK_LPW                0x4B
#define CMD_STATUS_OPENED_BUTTON            0x4C
#define CMD_STATUS_NOT_STOLEN               0x60

#define CMD_TESTMODE_DATA_REQ               0x50
#define CMD_TESTMODE_DATA_RESP              0x51
#define CMD_TESTMODE_RESULT                 0x52 
#define CMD_TESTMODE_SERIAL                 0x53
#define CMD_TESTMODE_SERIAL_PRO             0x54

#define CMD_ACK                             0xAC

#define CMD_ERR_WRONG_CRC                   0xE0
#define CMD_ERR_WRONG_COMMAND               0xE1
#define CMD_ERR_WRONG_LENGTH                0xE2
#define CMD_ERR_AUTH_FAILED                 0xE3
#define CMD_ERR_MCP_INIT_FAILED             0xE4
#define CMD_ERR_NOT_SUPPORTED               0xE5

#define CMD_FW_SEND_PACKET                  0xF0
#define CMD_FW_REQ_PACKET                   0xF1
#define CMD_FW_FINISHED                     0xF2
#define CMD_FW_UPDATE_SUCCESSFUL            0xF3
#define CMD_FW_UPDATE_FAILED                0xF4


#endif
