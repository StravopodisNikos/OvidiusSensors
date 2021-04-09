#define TOTAL_SENSORS_USED      4
#define TOTAL_ACTIVE_JOINTS     4       // [9-4-21] yet!
#define DUE_MAX_BITS_RESOL      12
#define DUE_MAX_ANAL_RESOLUTION 4096
#define MEGA_ANAL_RESOLUTION    1023
#define MEGA_UNO_mV             5000
#define DUE_mV                  3300
#define CUR_VOL_DIV_OVER        200      // since 10K and 20K R were used max vol divider output is 3500
#define DUE_CLOCK_Hz_MAX        84000000 // [Hz]

// GRIPPER SERVO
#define OPEN_GRIPPER_POSITION 	0
#define CLOSE_GRIPPER_POSITION	175
#define SERVO_STEP              2
#define GRIPPER_SERVO_PIN       12
// GRIPPER FSR
#define FSR_ANAL_PIN1           A3
#define FSR_ANAL_PIN2           A4
#define GRIPPER_MOV_DEL_MILLIS  25
#define _FSR_VCC_mV             5000
#define _FSR_VCC_DUE_mV         3300
#define _FSR_R_Ohm              10000
#define _FSR_R_microOhm         1000000
#define MAX_GRASP_FORCE         10
#define MIN_GRASP_FORCE         2

//#define _FSR_CALIBR_OFFSET      100

// FORCE SENSOR
#define num_FORCE_SENSORS       3
#define TIMES_FOR_MEASURE       1
#define DOUT_PIN_X              11 // 38
#define SCK_PIN_X               12 // 39
#define DOUT_PIN_Y              32
#define SCK_PIN_Y               33
#define DOUT_PIN_Z              42
#define SCK_PIN_Z               43
#define SENSOR_TIMEOUT          500
#define TIMEOUT_DELAY_MS        5

// IMU
#define FILTER_UPDATE_RATE_HZ   250
//#define PRINT_EVERY_N_UPDATES   100
//#define FILTER_INTERVAL_MILLIS_LOW_FREQ     250     // @4Hz
#define GYR_ID  1
#define ACC_ID  2
#define MAG_ID  3

// SD card read/write
#define MAX_LOG_DATA            20    // [9-4-21]  IS USED FOR STATIC MEMORY PREALLOCATION FOR THE TOTAL DATA TO BE LOGGED DURING EACH TASK
#define MAX_DATA_LENGTH         10000 // [31-3-21] NEEDS TESTING - THIS VAUE SHOULD BE BIGGER THAN THE MAX BYTES WRITTEN IN A SESSION FILE
#define SESSION_DIR_CHAR_LEN    10
#define SENSOR_DIR_CHAR_LEN     15
#define LOG_FILES_DIR_CHAR_LEN  20
#define SD_CARD_CS_PIN          53
#define SD_INIT_TIMEOUT_MILLIS  1000
#define TIME_HEADER             "T"   
#define TIME_REQUEST             7     
#define SD_STABIL_MILLIS         5
// Log files ID
#define POS_LOG_ID      0
#define VEL_LOG_ID      1
#define FOR_LOG_ID      2
#define CUR_LOG_ID      3
#define IMU_LOG_ID      4

//CURRENT SENSOR ADAFRUIT
#define CUR_SENSOR_INIT_TIMEOUT_MILLIS 5000
#define MAX_CURRENT_mA 3200

// CURRENT SENSOR ACS712
#define ACS_VOLTAGE_READ_PIN    A5
#define ACS_VOLTAGE_START       2.5f
#define ACS_VOLTAGE_IN          5.0f   // [V]
#define ACS_tr_1nF_micros       8
#define ACS_30A_SENSITIVITY     0.066f // [V/A]
#define STP_CURRENT_LIMIT       5.0f   // [A]
#define nCurSmaples             50