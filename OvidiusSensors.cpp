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

bool force3axis::setupForceSensor(HX711 *ptr2hx711, float manual_calibration_scale, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error)
{
    /*
     * Initializes pins - sets scale factor extracted manually from off-assembly
     * installation and changes state OFF->READY
     */
    ptr2hx711 = & ForceSensorAxis;

    // initialize sensor pins
    ptr2hx711->begin(_DOUT_PIN_AXIS, _SCK_PIN_AXIS);

    // check if force sensor is connected
    _fn_state = ptr2hx711->wait_ready_timeout(_SENSOR_TIMEOUT, _TIMEOUT_DELAY_MS);
    //_fn_state = true;
    if (_fn_state)
    {
        (* debug_error) = NO_ERROR;
        ptr2hx711->set_scale(manual_calibration_scale);       // sets scale to calibration factor given for each sensor
        ptr2hx711->tare(TIMES_FOR_MEASURE);             // sets OFFSET 
    }
    else
    {
        (* debug_error) = TIMEOUT_ERROR;
    }

    if ((* debug_error) == NO_ERROR)
    {
        _force_state = FORCE_READY;

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

bool force3axis::getPermanentZeroOffset(HX711 * ptr2hx711, long * axis_offset)
{
    /*
     *  Just to be able to see the offset values calculated after assembly
     */
    ptr2hx711 = & ForceSensorAxis;

    if ( _force_state == FORCE_READY )
    {
        // get the value measured
        (*axis_offset) = ptr2hx711->get_offset();
        return true;
    }
    else
    {
        return false;
    }
}

bool force3axis::measureForceKilos(HX711 * ptr2hx711, float * force_measurements_kgs, debug_error_type * debug_error)
{
    // 

    ptr2hx711 = & ForceSensorAxis;

    if ( _force_state == FORCE_READY )
    {
        (* debug_error) = NO_ERROR;

        // takes measurement: in order to use get_units SCALE+OFFSET must be set
        (* force_measurements_kgs) = ptr2hx711->get_units(TIMES_FOR_MEASURE);

        return true;
    }
    else
    {
        (* debug_error) = STATE_NOT_READY;

        return false;
    }  
}

bool force3axis::getRawMeasurement(HX711 * ptr2hx711, float * raw_measurement, debug_error_type * debug_error)
{
    // 

    ptr2hx711 = & ForceSensorAxis;

    if ( _force_state == FORCE_READY )
    {
        (* debug_error) = NO_ERROR;

        // takes rawvmeasurement
        (* raw_measurement) = ptr2hx711->read_average(TIMES_FOR_MEASURE);

        return true;
    }
    else
    {
        (* debug_error) = STATE_NOT_READY;

        return false;
    }  
}

void force3axis::setIdleState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error)
{
    ptr2hx711 = & ForceSensorAxis;

    if ( _force_state == FORCE_READY )
    {
        (* debug_error) = NO_ERROR;

        ptr2hx711->power_down();

        _force_state = FORCE_IDLE;

        *force_current_state =  _force_state; 

    }
    else
    {
        (* debug_error) = STATE_NOT_READY;

        _force_state == FORCE_ERROR;

        *force_current_state =  _force_state;      
    }
}

void force3axis::setReadyState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error)
{
    ptr2hx711 = & ForceSensorAxis;

    if ( _force_state == FORCE_IDLE )
    {
        (* debug_error) = NO_ERROR;

        ptr2hx711->power_up();

        _force_state = FORCE_READY;

        *force_current_state =  _force_state; 

    }
    else
    {
        (* debug_error) = STATE_NOT_IDLE;

        _force_state == FORCE_ERROR;

        *force_current_state =  _force_state;      
    }
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