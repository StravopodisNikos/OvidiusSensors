// GRIPPER SERVO
#define OPEN_GRIPPER_POSITION 	0
#define CLOSE_GRIPPER_POSITION	175
#define SERVO_STEP              2
#define GRIPPER_SERVO_PIN       12
// GRIPPER FSR
#define FSR_ANAL_PIN1           A3
#define FSR_ANAL_PIN2           A4
#define _FSR_VCC_mV             5000
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
#define SD_CARD_CS_PIN          53
#define SD_INIT_TIMEOUT_MILLIS  10000
#define TIME_HEADER             "T"   
#define TIME_REQUEST             7     
#define SD_STABIL_MILLIS         5
// Log files ID
#define POS_LOG_ID      0
#define VEL_LOG_ID      1
#define FOR_LOG_ID      2
#define CUR_LOG_ID      3
#define IMU_LOG_ID      4