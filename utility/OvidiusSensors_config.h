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
#define TIMES_FOR_MEASURE       5
#define DOUT_PIN_X              38
#define SCK_PIN_X               39
#define DOUT_PIN_Y              32
#define SCK_PIN_Y               33
#define DOUT_PIN_Z              42
#define SCK_PIN_Z               43
#define SENSOR_TIMEOUT          500
#define TIMEOUT_DELAY_MS        5
