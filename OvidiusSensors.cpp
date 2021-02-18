#include "Arduino.h"
#include <EEPROM.h>
#include <Servo.h>
#include "HX711.h"

#include <OvidiusSensors.h>
#include <utility/OvidiusSensors_config.h>
#include <utility/OvidiusSensors_debug.h>
#include <ovidius_robot_controller_eeprom_addresses.h>

using namespace sensors;
force3axis::force3axis(byte DOUT_PIN,byte SCK_PIN): ForceSensorAxis()
{
    _DOUT_PIN_AXIS = DOUT_PIN;
    _SCK_PIN_AXIS  = SCK_PIN;

    _SENSOR_TIMEOUT   = SENSOR_TIMEOUT;
    _TIMEOUT_DELAY_MS = TIMEOUT_DELAY_MS;
}

bool force3axis::setupForceSensor(HX711 *ptr2hx711, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error)
{
    ptr2hx711 = & ForceSensorAxis;

    // check if force sensor is connected
    _fn_state = ptr2hx711->wait_ready_timeout(_SENSOR_TIMEOUT, _TIMEOUT_DELAY_MS);
    //_fn_state = true;
    if (_fn_state)
    {
        ptr2hx711->begin(_DOUT_PIN_AXIS, _SCK_PIN_AXIS);
        (* debug_error) = NO_ERROR;
    }
    else
    {
        (* debug_error) = TIMEOUT_ERROR;
    }

    if ((* debug_error) == NO_ERROR)
    {
        _force_state = FORCE_IDLE;

        *force_current_state =  _force_state;

        return true;
    }
    else
    {
        _force_state = FORCE_OFF;

        *force_current_state =  _force_state;        
        return false;
    }
}

bool force3axis::calibrateForceSensor(HX711 *ptr2hx711, debug_error_type * debug_error, float * calibration_factor)
{
    // UNDER DEVEL!!! 

    ptr2hx711 = & ForceSensorAxis;

    // reset scale to 0
    ptr2hx711->set_scale();

    ptr2hx711->tare();

    // start calibration
    bool calibration_finished = true;

    do
    {
                
    } while (!calibration_finished);

    return true;
}

float force3axis::measureForceNewtons(HX711 * ptr2hx711, byte times_measured, float * accel_tool_dir)
{
    // UNDER DEVEL!!! 

    // must pre-specify the acceleration in tool space direction the force is measured!

    ptr2hx711 = & ForceSensorAxis;

    float measurement_avg;

    float measurement_avg_nwt;

    measurement_avg = ptr2hx711->get_units(times_measured);

    measurement_avg_nwt = measurement_avg * (*accel_tool_dir);

    return measurement_avg_nwt;
}

float force3axis::measureRaw(HX711 * ptr2hx711, byte times_measured)
{
    ptr2hx711 = & ForceSensorAxis;

    float measurement_avg;

    measurement_avg = ptr2hx711->get_units(times_measured);

    return measurement_avg;
}

// ============================================================================

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

    _gripper_state = GRIPPER_OPENED;

    *gripper_current_state =  _gripper_state;
}

void gripper::closeGripper(Servo * ptr2servo, tools::gripper_states * gripper_current_state)
{
    ptr2servo = &GripperServo;

    ptr2servo->attach(_SERVO_PWM_PIN);

    ptr2servo->write(CLOSE_GRIPPER_POSITION);

    //ptr2servo->detach();

    _gripper_state = GRIPPER_CLOSED;

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

    } while ( (!object_gripped) && (!impossible_grip) );
    
    //ptr2servo->detach();

    _gripper_state = GRIPPER_CLOSED;

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

void gripper::readGripperStateEEPROM(tools::gripper_states * gripper_current_state)
{
    /*
     *  Reads gripper state from EEPROM - Executed @ setup
     */

   EEPROM.get(CS_GRIPPER_EEPROM_ADDR, *gripper_current_state);

   _gripper_state = (* gripper_current_state);

}

void gripper::writeGripperStateEEPROM(tools::gripper_states * gripper_current_state)
{
    /*
     *  Writes gripper state from EEPROM - Executed @setup if user wants
     */

   EEPROM.put(CS_GRIPPER_EEPROM_ADDR, *gripper_current_state);
}