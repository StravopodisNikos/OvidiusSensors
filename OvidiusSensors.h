 /*
  * OvidiusSensors.h  - Library for interfacing with sensors-tools of Ovidius Manipulator
  * Created by N.A. Stravopodis, February, 2021.
  */

#ifndef OvidiusSensors_h
#define OvidiusSensors_h
#include "Arduino.h"
#include <stdlib.h>
/*
 *  EXTERNAL LIBRARIES
 */

#include <Servo.h>                // GRIPPER LIBRARY
#include "HX711.h"                // FORCE SENSOR LIBRARY
//#include "SensorFusion.h"         // IMU LIBRARIES - include sensor fusion+filter
//#include <Adafruit_FXAS21002C.h>  // include gyro
//#include <Adafruit_FXOS8700.h>    // include accel+mag
//#include <Adafruit_Sensor.h>
//#include "Adafruit_Sensor_Calibration.h"
//#include "Adafruit_Sensor_Calibration_EEPROM.h"
//#include <Adafruit_AHRS.h>
//#include <Wire.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SPI.h>
#include <SD.h>
//#include <SdFat.h>
#include <TimeLib.h>

enum function_exec_state {success, failed};
typedef unsigned char debug_error_type;

namespace sensors
{
  typedef enum sensors_list {FORCE_3AXIS, IMU_9AXIS, CURRENT_JOINT1, JOINT_POS, JOINT_VEL};
  typedef enum force_sensor_states {FORCE_OFF,FORCE_IDLE,FORCE_READY,FORCE_READS,FORCE_WRITES,FORCE_ERROR};
  typedef enum imu_sensor_states {IMU_READY,IMU_BUSY,IMU_ERROR};
  typedef enum imu_filter {MAHONY_F,MADGWICK_F};
/*
  struct imu_packet
  {
    Adafruit_FXAS21002C *ptr2_fxas;   // is used to point to gyro object creted in ino file
    Adafruit_FXOS8700   *ptr2_fxos;   // is used to point to accel/mag object creted in ino file
    //SF                  *ptr2_filter; // removed 2/3/21 - will implement original adafruit class
    Adafruit_Madgwick   *ptr2_filter;         // added 2/3/21 - original adafruit
    Adafruit_Sensor_Calibration_EEPROM *cal;  // added 2/3/21 - original adafruit

    sensors_event_t      *gyr_event, *acc_event, *mag_event;
    //sensors_event_t      gyr_event, acc_event, mag_event;

    float  roll_c;  // will store current angles (current = measured+filtered!)
    float  pitch_c;
    float  yaw_c; 
  };
  */

  struct current_packet
  {
    Adafruit_INA219 * ptr2ina219;
    double current_measurement_mA;
  };

class force3axis
{
  private:
    byte _DOUT_PIN_AXIS;
    byte _SCK_PIN_AXIS;

    unsigned long _SENSOR_TIMEOUT;
    unsigned long _TIMEOUT_DELAY_MS;

    bool _fn_state;

    force_sensor_states _force_state;

    void setIdleState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state);

    void setReadyState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state);

    void getCurrentState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error);

  public:

    //force_sensor_states _force_state;         // made public in order to be accessed by other classes

    force3axis(byte DOUT_PIN,byte SCK_PIN);

    bool setupForceSensor(HX711 *ptr2hx711, float manual_calibration_scale, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error);

    bool getPermanentZeroOffset(HX711 * ptr2hx711, long * axis_offset);

    bool measureForceKilos(HX711 * ptr2hx711, float * force_measurements_kgs, debug_error_type * debug_error);

    bool getRawMeasurement(HX711 * ptr2hx711, float * raw_measurement, debug_error_type * debug_error);

    void setIdleState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error);

    void setReadyState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error);

    HX711 ForceSensorAxis;
};

/*
class imu9dof: public Adafruit_FXAS21002C, public Adafruit_FXOS8700, public SF, public Adafruit_Sensor_Calibration_EEPROM
{
  private:
    imu_sensor_states _imu_state;

    int32_t _GYR_ID;
    int32_t _ACC_ID;
    int32_t _MAG_ID;

    gyroRange_t _GYR_RANGE;
    fxos8700AccelRange_t _ACC_RANGE;

    int _FILTER_UPDATE_RATE_MILLIS;

    unsigned long _last_filter_update;

  public:
    imu9dof(gyroRange_t gyr_range, fxos8700AccelRange_t acc_range, int filter_millis_interval, int32_t gyr_id, int32_t acc_id, int32_t mag_id);

    bool setupIMU(imu_packet * ptr2imu_packet, sensors::imu_sensor_states * imu_current_state, debug_error_type * debug_error);

    bool setupIMU2(imu_packet * ptr2imu_packet, sensors::imu_sensor_states * imu_current_state, debug_error_type * debug_error);

    bool measure_with_filter_IMU(imu_packet * ptr2imu_packet, sensors::imu_filter FILTER_SELECT, debug_error_type * debug_error);

    bool measure_with_filter_IMU2(imu_packet * ptr2imu_packet, sensors::imu_filter FILTER_SELECT, debug_error_type * debug_error);

    void getCurrentState(sensors::imu_sensor_states * imu_current_state, debug_error_type * debug_error);

    void setCurrentState(sensors::imu_sensor_states * imu_current_state);

    void getFilterInterval(int * filter_interval);

    imu_packet IMU_PACKET;

};
*/

class currentSensor: public Adafruit_INA219
{
  private:
    unsigned long _last_current_update;
    int _analog_voltage_measurement;
    int _voltage_mapped;
    float _voltage_mapped_V;
    double _acs712_VIN;
    double _acs712_Vstart;
    
  public:
    currentSensor();
    //~currentSensor();

    // ADAFRUIT INA219
    void setupCurrentSensor(current_packet * ptr2cur_packet, debug_error_type * debug_error);

    void measureCurrent_mA(current_packet * ptr2cur_packet , debug_error_type * debug_error);

    // ALLEGRO ACS712
    void measureCurrentACS712_A(float & current_measurement, debug_error_type * debug_error);

};

}

namespace tools
{
  typedef enum gripper_states {GRIPPER_CLOSED,GRIPPER_OPENED};

  class gripper
  {
    public:
      gripper(uint8_t servo_pwm_pin, uint8_t fsr_analog_input_pin);

      void openGripper(Servo * ptr2servo, tools::gripper_states * gripper_current_state);

      void closeGripper(Servo * ptr2servo, tools::gripper_states * gripper_current_state);

      void closeGripperForce(Servo * ptr2servo, unsigned long grasp_force_limit, tools::gripper_states * gripper_current_state);

      unsigned long measureForce();

      int setupGripper();

      //void readGripperStateEEPROM(tools::gripper_states * gripper_current_state);

      //void writeGripperStateEEPROM(tools::gripper_states * gripper_current_state);

      Servo GripperServo;
      
    private:
      byte _SERVO_PWM_PIN;
      byte _FSR_AI_PIN;
      int _FSR_READING;
      int _FSR_VOLTAGE;
      int _FSR_CALIBR_OFFSET;
      unsigned long _FSR_RESISTANCE;
      unsigned long _FSR_CONDUCTANCE;
      unsigned long _FSR_FORCE;

      gripper_states _gripper_state;

  };

  class dataLogger: public String, public File, public SDClass
  {
    private:
      uint8_t _file_response;
      boolean _boolean_sd_response;

    public:
      dataLogger();
      //~dataLogger();
      void setupDataLogger(File *ptr2root, debug_error_type * debug_error);
      
      //bool createSessionDir(String &session_dir);
      bool createSessionDir( char * session_dir);

      //bool createSensorDir(sensors::sensors_list sensor_choice, String session_dir, String &final_sensor_dir);
      bool createSensorDir(sensors::sensors_list sensor_choice, const char * session_dir, char * final_sensor_dir, debug_error_type * debug_error);

      //void createFile(File *ptr2file, String final_sensor_dir, String &filename , byte OPERATION,  debug_error_type * debug_error);      
      //void createFile(String final_sensor_dir, String &filename ,  debug_error_type * debug_error);
      //void createFile(SdFile *ptr2file, String &filename ,  debug_error_type * debug_error);
      void createFile(File *ptr2file, const char * path2file,  char * filename ,  debug_error_type * debug_error);

      //void openFile(SdFile *ptr2file, String filename , byte OPERATION,  debug_error_type * debug_error);
      void openFile(File *ptr2file, const char * filename , uint8_t OPERATION,  debug_error_type * debug_error);

      void closeFile(File *ptr2file, debug_error_type * debug_error);

      //template <class T>
      //void writeData(T data2write, unsigned long timestamp, unsigned long data_cnt, File *ptr2file, debug_error_type * debug_error);
      void writeData(float * data2write, int size, unsigned long timestamp, unsigned long data_cnt, File *ptr2file, debug_error_type * debug_error);
  };
  
}

#endif