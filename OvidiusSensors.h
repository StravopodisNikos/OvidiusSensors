 /*
  * OvidiusSensors.h  - Library for interfacing with sensors-tools of Ovidius Manipulator
  * Created by N.A. Stravopodis, February, 2021.
  */

#ifndef OvidiusSensors_h
#define OvidiusSensors_h

#include "Arduino.h"

#include <Servo.h>
//#include "HX711.h"

enum function_exec_state {success, failed};


namespace sensors
{
/*
class force3axis:
{
  private:
    //HX711 &ptr2hx711;
    const int _sensor_dout_pin = SENSOR_DOUT_PIN;
    const int _sensor_sck_pin  = SENSOR_SCK_PIN;
  public:
    bool InitForceSensor();

}
*/

/*
class imu:
{
};

#endif
*/
}

namespace tools
{
  typedef enum gripper_states {CLOSED,OPENED};

  class gripper
  {
    public:
      gripper(uint8_t servo_pwm_pin, uint8_t fsr_analog_input_pin);

      void openGripper(Servo * ptr2servo, tools::gripper_states * gripper_current_state);

      void closeGripper(Servo * ptr2servo, tools::gripper_states * gripper_current_state);

      void closeGripperForce(Servo * ptr2servo, unsigned long grasp_force_limit, tools::gripper_states * gripper_current_state);

      unsigned long measureForce();

      int setupGripper();

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