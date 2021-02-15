#include "Arduino.h"

#include <Servo.h>
//#include "HX711.h"

#include <OvidiusSensors.h>
#include <utility/OvidiusSensors_config.h>

/*
force3axis::force3axis()
{

}

force3axis::InitForceSensor(HX711 *ptr2hx711, )
{
    ptr2hx711->begin()
}
*/
using namespace tools;

gripper::gripper(uint8_t servo_pwm_pin, uint8_t fsr_analog_input_pin): GripperServo()
{
    _SERVO_PWM_PIN = servo_pwm_pin;
    _FSR_AI_PIN    = fsr_analog_input_pin;

    // no Servo function can be executed here!
}

void gripper::openGripper(Servo * ptr2servo, tools::gripper_states * gripper_current_state)
{
    ptr2servo = &GripperServo;

    ptr2servo->attach(_SERVO_PWM_PIN);

    ptr2servo->write(OPEN_GRIPPER_POSITION);

    //ptr2servo->detach();

    _gripper_state = OPENED;

    *gripper_current_state =  _gripper_state;
}

void gripper::closeGripper(Servo * ptr2servo, tools::gripper_states * gripper_current_state)
{
    ptr2servo = &GripperServo;

    ptr2servo->attach(_SERVO_PWM_PIN);

    ptr2servo->write(CLOSE_GRIPPER_POSITION);

    //ptr2servo->detach();

    _gripper_state = CLOSED;

    *gripper_current_state =  _gripper_state;
}

void gripper::closeGripperForce(Servo * ptr2servo, unsigned long grasp_force_limit, tools::gripper_states * gripper_current_state)
{
    unsigned long returned_grasp_force;

    ptr2servo = &GripperServo;

    ptr2servo->attach(_SERVO_PWM_PIN);

    // suppose that gripper is in open position!
    int gripper_position = OPEN_GRIPPER_POSITION;

    bool object_gripped = false;
    bool impossible_grip = false;

    do
    {
        // start closing gripper
        gripper_position += SERVO_STEP;
        ptr2servo->write(gripper_position);

        // measure force
        returned_grasp_force = gripper::measureForce();
        
        // change flag
        if (returned_grasp_force >= grasp_force_limit)
        {
            object_gripped = true;
        }

        if (gripper_position > CLOSE_GRIPPER_POSITION)
        {
            impossible_grip = true;
        }
        
        delay(50);
        //Serial.print("Grasp Force="); Serial.println(returned_grasp_force);
        //Serial.print("Gripper Pos="); Serial.println(gripper_position);

    } while ( (!object_gripped) && (!impossible_grip) );
    
    //ptr2servo->detach();

    _gripper_state = CLOSED;

    *gripper_current_state =  _gripper_state;
}

unsigned long gripper::measureForce()
{
    // Returns long value in [N]
    unsigned long force_measured;

    _FSR_READING = analogRead(_FSR_AI_PIN); 

    _FSR_VOLTAGE = map(_FSR_READING, 0, 1023, 0, _FSR_VCC_mV);

    if ( _FSR_VOLTAGE < _FSR_CALIBR_OFFSET)
    {
        force_measured = 0;
    }
    else
    {
        /*
            Vmeasure = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
            FSR      = ((Vcc - V) * R) / V 
        */
       _FSR_RESISTANCE  = _FSR_VCC_mV - _FSR_VOLTAGE;
       _FSR_RESISTANCE *= _FSR_R_Ohm;
       _FSR_RESISTANCE /= _FSR_VOLTAGE;

       _FSR_CONDUCTANCE  =  _FSR_R_microOhm;
       _FSR_CONDUCTANCE /= _FSR_RESISTANCE;

       /*
            FSR guide graphs are used to approximate the force
        */

        if (_FSR_CONDUCTANCE <= 1000) {
          _FSR_FORCE = _FSR_CONDUCTANCE / 80;     
        } else {
          _FSR_FORCE = _FSR_CONDUCTANCE - 1000;
          _FSR_FORCE /= 30;          
        }

        force_measured = _FSR_FORCE;
    }

    return force_measured;
    
}

int gripper::setupGripper()
{
    _FSR_READING = analogRead(_FSR_AI_PIN);

    _FSR_VOLTAGE = map(_FSR_READING, 0, 1023, 0, _FSR_VCC_mV);

    _FSR_CALIBR_OFFSET = _FSR_VOLTAGE;

    int calibrationFsrVoltage = _FSR_VOLTAGE;

    return calibrationFsrVoltage;
}