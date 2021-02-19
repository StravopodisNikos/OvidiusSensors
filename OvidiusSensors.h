 /*
  * OvidiusSensors.h  - Library for interfacing with sensors-tools of Ovidius Manipulator
  * Created by N.A. Stravopodis, February, 2021.
  */

#ifndef OvidiusSensors_h
#define OvidiusSensors_h

#include "Arduino.h"

#include <Servo.h>
#include "HX711.h"

enum function_exec_state {success, failed};
typedef unsigned char debug_error_type;

namespace sensors
{
  typedef enum force_sensor_states {FORCE_OFF,FORCE_IDLE,FORCE_READY,FORCE_READS,FORCE_WRITES,FORCE_ERROR};

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

  public:

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
class imu:
{
};

#endif
*/
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

      void readGripperStateEEPROM(tools::gripper_states * gripper_current_state);

      void writeGripperStateEEPROM(tools::gripper_states * gripper_current_state);

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

}

#endif