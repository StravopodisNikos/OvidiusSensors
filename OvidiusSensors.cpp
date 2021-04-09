#include "Arduino.h"
#include <stdlib.h>
#include <math.h> 
//#include <EEPROM.h>
#include <Servo.h>
#include "HX711.h"
#include <SPI.h>
#include <SD.h> 
//#include <SdFat.h>
#include <TimeLib.h>
#include <OvidiusSensors.h>
#include <utility/OvidiusSensors_config.h>
#include <utility/OvidiusSensors_debug.h>
#include <utility/sd_ct_chars.h>
#include <ovidius_robot_controller_eeprom_addresses.h>

using namespace sensors;
force3axis::force3axis(byte DOUT_PIN,byte SCK_PIN)//: ForceSensorAxis()
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
    //ptr2hx711 = & ForceSensorAxis;

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
    //ptr2hx711 = & ForceSensorAxis;

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
    // modified to execute for all calls

    //ptr2hx711 = & ForceSensorAxis;

    //_force_state = FORCE_READY;
    getCurrentState(ptr2hx711, &_force_state, debug_error);                     // seems redundant but enables calling fn outside class and reading current state

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

    //ptr2hx711 = & ForceSensorAxis;

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
    //ptr2hx711 = & ForceSensorAxis;

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
    //ptr2hx711 = & ForceSensorAxis;

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

void force3axis::getCurrentState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error)
{
    //ptr2hx711 = & ForceSensorAxis;

    *force_current_state = _force_state;
    
    switch (*force_current_state)
    {
        case FORCE_OFF:
            * debug_error = NO_ERROR;
            break;
        case FORCE_IDLE:
            * debug_error = NO_ERROR;
            break;
        case FORCE_READY:
            * debug_error = NO_ERROR;
            break;
        case FORCE_READS:
            * debug_error = NO_ERROR;
            break;
        case FORCE_WRITES:
            * debug_error = NO_ERROR;
            break;
        case FORCE_ERROR:
            * debug_error = NO_ERROR;
            break;       
        default:
            * debug_error = UNKNOWN_STATE;
            break;
    }

}
// ============================================================================
//  A D A F R U I T -- 9 D O F -- I M U -- W I T H -- F U S I O N
// ============================================================================
/*
imu9dof::imu9dof(gyroRange_t gyr_range, fxos8700AccelRange_t acc_range, int filter_change_rate_hz, int32_t gyr_id, int32_t acc_id, int32_t mag_id): Adafruit_FXAS21002C(gyr_id), Adafruit_FXOS8700(acc_id, mag_id), SF(), Adafruit_Sensor_Calibration_EEPROM()
{
    _GYR_ID = gyr_id;
    _ACC_ID = acc_id;
    _MAG_ID = mag_id;

    _GYR_RANGE = gyr_range;
    _ACC_RANGE = acc_range;

    _FILTER_UPDATE_RATE_MILLIS = filter_change_rate_hz;

    _last_filter_update = 0;
}

bool imu9dof::setupIMU(imu_packet * ptr2imu_packet, sensors::imu_sensor_states * imu_current_state, debug_error_type * debug_error)
{
    // begins imu sensor and sets state to ready

    // begin the gyroscope
    if( ! (ptr2imu_packet->ptr2_fxas->begin(_GYR_RANGE)) )  
    {
        *debug_error = GYRO_NOT_FOUND;
        while (1);        
    }
    // begin accel+mag
    if( ! (ptr2imu_packet->ptr2_fxos->begin(_ACC_RANGE)) )  
    {
        *debug_error = ACCEL_MAG_NOT_FOUND;
        while (1);        
    }

    if( !(*debug_error == GYRO_NOT_FOUND ) && !(*debug_error == ACCEL_MAG_NOT_FOUND ) )
    {
        *debug_error = NO_ERROR;

        _imu_state = IMU_READY;
        *imu_current_state = _imu_state;
        return true;
    }
    else
    {
        _imu_state = IMU_ERROR;
        *imu_current_state = _imu_state;
        return false;
    }
}

bool imu9dof::setupIMU2(imu_packet * ptr2imu_packet, sensors::imu_sensor_states * imu_current_state, debug_error_type * debug_error)
{
    // copid from setupIMU. works with main Adafruit libraries
    // that implement calibration

    // loads calibration data for sensor used
    ptr2imu_packet->cal->loadCalibration();
    ptr2imu_packet->cal->printSavedCalibration();

    // init sensors
    // begins imu sensor and sets state to ready
    // begin the gyroscope
    if( ! (ptr2imu_packet->ptr2_fxas->begin(_GYR_RANGE)) )  
    {
        *debug_error = GYRO_NOT_FOUND;
        while (1);        
    }
    // begin accel+mag
    if( ! (ptr2imu_packet->ptr2_fxos->begin(_ACC_RANGE)) )  
    {
        *debug_error = ACCEL_MAG_NOT_FOUND;
        while (1);        
    }

    // begin the filter
    ptr2imu_packet->ptr2_filter->begin(FILTER_UPDATE_RATE_HZ);

    //Wire.setClock(400000);          // 400KHz as in calibrated orientation

    if( !(*debug_error == GYRO_NOT_FOUND ) && !(*debug_error == ACCEL_MAG_NOT_FOUND ) )
    {
        *debug_error = NO_ERROR;

        _imu_state = IMU_READY;
        *imu_current_state = _imu_state;
        return true;
    }
    else
    {
        _imu_state = IMU_ERROR;
        *imu_current_state = _imu_state;
        return false;
    }
}

bool imu9dof::measure_with_filter_IMU(imu_packet * ptr2imu_packet, sensors::imu_filter FILTER_SELECT, debug_error_type * debug_error)
{
    // This function must always be called inside if statement
    // for state-machine implementation

    // measurement data variables
    float ax,ay,az,gx,gy,gz,mx,my,mz;
    float time_step;

    imu9dof::getCurrentState(&_imu_state, debug_error);                     // seems redundant but enables calling fn outside class and reading current state

    if ( _imu_state == IMU_READY )
    {
        (* debug_error) = NO_ERROR;

        _imu_state = IMU_BUSY;
        
        // Get sensor event
        ptr2imu_packet->ptr2_fxas->getEvent(ptr2imu_packet->gyr_event);
        ptr2imu_packet->ptr2_fxos->getEvent(ptr2imu_packet->acc_event, ptr2imu_packet->mag_event );

        ax = ptr2imu_packet->acc_event->acceleration.x;  // m/s2
        ay = ptr2imu_packet->acc_event->acceleration.y;
        az = ptr2imu_packet->acc_event->acceleration.z;
        // units conversion: [m/s2] -> [g]
        //ax = 0.101972f * ax;        // g
        //ay = 0.101972f * ay;
        //az = 0.101972f * az;

        mx = ptr2imu_packet->mag_event->magnetic.x;      // uT
        my = ptr2imu_packet->mag_event->magnetic.y;
        mz = ptr2imu_packet->mag_event->magnetic.z;
        // units conversion: [uT] -> [G]
        //mx   = 100.0f * mx;         // G:flux
        //my   = 100.0f * my;
        //mz   = 100.0f * mz;
        gx = ptr2imu_packet->gyr_event->gyro.x;          // r/s
        gy = ptr2imu_packet->gyr_event->gyro.y;
        gz = ptr2imu_packet->gyr_event->gyro.z;

        // compute integration step for filter update
        time_step = deltatUpdate();

        // choose filter
        switch (FILTER_SELECT)
        {
            case MAHONY_F:
                MahonyUpdate(gx, gy, gz, ax, ay, az, time_step);
                break;
            case MADGWICK_F:
                MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, time_step);
                break;    
            default:
                *debug_error = INCORRECT_FILTER_SELECTION;
                break;
        }

        // finally get angles
        ptr2imu_packet->pitch_c = getPitch();
        ptr2imu_packet->roll_c  = getRoll();
        ptr2imu_packet->yaw_c  = getYaw();

        _imu_state = IMU_READY;
        return true;
    }
    else
    {
        (* debug_error) = STATE_NOT_READY;

        return false;
    } 
    
}

bool imu9dof::measure_with_filter_IMU2(imu_packet * ptr2imu_packet, sensors::imu_filter FILTER_SELECT, debug_error_type * debug_error)
{
    // Copied from measure_with_filter_IMU. But implements only Madgwick
    // with calibration data accessed from EEPROM!
    // This function must always be called inside if statement
    // for state-machine implementation

    // measurement data variables
    float ax,ay,az,gx,gy,gz,mx,my,mz;
    float time_step;

    imu9dof::getCurrentState(&_imu_state, debug_error);                     // seems redundant but enables calling fn outside class and reading current state

    if ( _imu_state == IMU_READY )
    {
        (* debug_error) = NO_ERROR;

        _imu_state = IMU_BUSY;
        
        // Get sensor event
        ptr2imu_packet->ptr2_fxas->getEvent(ptr2imu_packet->gyr_event);
        ptr2imu_packet->ptr2_fxos->getEvent(ptr2imu_packet->acc_event, ptr2imu_packet->mag_event );

        ax = ptr2imu_packet->acc_event->acceleration.x;  // m/s2
        ay = ptr2imu_packet->acc_event->acceleration.y;
        az = ptr2imu_packet->acc_event->acceleration.z;
        // calibrate accelerometer data -> TO DO! [2/3/21]

        // units conversion: [m/s2] -> [g]
        //ax = 0.101972f * ax;        // g
        //ay = 0.101972f * ay;
        //az = 0.101972f * az;

        // calibrate magnetometer data -> ready
        ptr2imu_packet->cal->Adafruit_Sensor_Calibration::calibrate(*(ptr2imu_packet->mag_event));

        mx = ptr2imu_packet->mag_event->magnetic.x;      // uT
        my = ptr2imu_packet->mag_event->magnetic.y;
        mz = ptr2imu_packet->mag_event->magnetic.z;

        // units conversion: [uT] -> [G]
        //mx   = 100.0f * mx;         // G:flux
        //my   = 100.0f * my;
        //mz   = 100.0f * mz;

        gx = ptr2imu_packet->gyr_event->gyro.x;          // r/s
        gy = ptr2imu_packet->gyr_event->gyro.y;
        gz = ptr2imu_packet->gyr_event->gyro.z;

        // calibrate gyroscope data -> TO DO! [2/3/21]

        gx = gx * SENSORS_RADS_TO_DPS;                   // r/s ->d/s for Adafruit_ahrs library   
        gy = gy * SENSORS_RADS_TO_DPS;
        gz = gz * SENSORS_RADS_TO_DPS;

        // implement filter
        ptr2imu_packet->ptr2_filter->update(gx, gy, gz, ax, ay, az, mx, my, mz);

        // finally get angles
        ptr2imu_packet->pitch_c = ptr2imu_packet->ptr2_filter->getPitch();
        ptr2imu_packet->roll_c  = ptr2imu_packet->ptr2_filter->getRoll();
        ptr2imu_packet->yaw_c  = ptr2imu_packet->ptr2_filter->getYaw();

        _imu_state = IMU_READY;
        return true;
    }
    else
    {
        (* debug_error) = STATE_NOT_READY;

        return false;
    } 
    
}

void imu9dof::getCurrentState(sensors::imu_sensor_states * imu_current_state, debug_error_type * debug_error)
{

    *imu_current_state = _imu_state;
    
    switch (*imu_current_state)
    {
        case IMU_READY:
            * debug_error = NO_ERROR;
            break;
        case IMU_BUSY:
            * debug_error = NO_ERROR;
            break;
        case IMU_ERROR:
            * debug_error = NO_ERROR;
            break;      
        default:
            * debug_error = UNKNOWN_STATE;
            break;
    }

}

void imu9dof::setCurrentState(sensors::imu_sensor_states * imu_current_state)
{
    _imu_state = *imu_current_state;
}

void imu9dof::getFilterInterval(int * filter_interval)
{
    *filter_interval = _FILTER_UPDATE_RATE_MILLIS;
}
*/


currentSensor::currentSensor(): Adafruit_INA219()
{
    // [24-3-21] Added functions for ACS712
    _last_current_update = 0;

    _analog_voltage_measurement = 0;

    _acs712_Vstart  = ACS_VOLTAGE_START;
    _acs712_VIN     = ACS_VOLTAGE_IN;
}
/*
currentSensor::~currentSensor()
{
}
*/
void currentSensor::setupCurrentSensor(current_packet * ptr2cur_packet, debug_error_type * debug_error)
{
    
    bool initialized_current_sensor = false;
    int total_time;
    unsigned long started_currentSensor_initialization = millis();
    do
    {
        initialized_current_sensor = ptr2cur_packet->ptr2ina219->begin();
        if (!initialized_current_sensor)
        {
           *debug_error = CUR_SENSOR_INIT_FAILED;
        }
        
        total_time = millis() - started_currentSensor_initialization;
    } while( (!initialized_current_sensor) && (total_time < CUR_SENSOR_INIT_TIMEOUT_MILLIS) );

    if (initialized_current_sensor)
    {
        *debug_error = NO_ERROR;
    }
    
    return;
}

void currentSensor::measureCurrent_mA(current_packet * ptr2cur_packet, debug_error_type * debug_error)
{
    ptr2cur_packet->current_measurement_mA = ptr2cur_packet->ptr2ina219->getCurrent_mA();

    if ( (ptr2cur_packet->current_measurement_mA < 0 ) || (ptr2cur_packet->current_measurement_mA > MAX_CURRENT_mA) )
    {
        *debug_error = CUR_SENSOR_WRONG_MEAS;
    }
    else
    {
        *debug_error = NO_ERROR;
    }
    
    return;
}

void currentSensor::measureCurrentACS712_A(float & current_measurement, debug_error_type * debug_error)
{
    // [30-3-2021] Since delay is used for sampling this function must be used AFTER the syncReadCurrent
    // and no further delay should be added. It must be doe because sensor is noisy...
    // [30-3-2021] How should the error characteristics be integrated? Vnoise = 35mV???
    
    int _sum_analog_voltage_measurement;
    int _final_analog_voltage_measurement;

    analogReadResolution(DUE_MAX_BITS_RESOL);         // FOR DUE ONLY -> SETS 12 bit resolution
    // Viout(analog) of the module+voltage divider circuit measured in arduino analog pin

    _sum_analog_voltage_measurement = 0;
    for (size_t i = 0; i < nCurSmaples; i++)
    {
        _analog_voltage_measurement = analogRead(ACS_VOLTAGE_READ_PIN);
        _sum_analog_voltage_measurement = _sum_analog_voltage_measurement + _analog_voltage_measurement;
        delayMicroseconds(ACS_tr_1nF_micros+2);
    }
    _final_analog_voltage_measurement = (int) (_sum_analog_voltage_measurement / nCurSmaples);
    
    // analog -> voltage
    //_voltage_calculated = (_analog_voltage_measurement / DUE_MAX_ANAL_RESOLUTION ) * DUE_mV;
    _voltage_mapped = map(_final_analog_voltage_measurement, 0, DUE_MAX_ANAL_RESOLUTION, 0, DUE_mV);
    _voltage_mapped_V =  float ( (_voltage_mapped) / 1000.0f) ;

    // direct current calculation formula
    current_measurement = ( _voltage_mapped_V - ACS_VOLTAGE_START) / ACS_30A_SENSITIVITY;
    current_measurement = fabs(current_measurement);

    if (current_measurement > STP_CURRENT_LIMIT)
    {
        * debug_error = HIGH_CURRENT_MEASURED;
    }
    else
    {
        * debug_error = NO_ERROR;
    }

    return;
}


using namespace tools;
// ============================================================================
//  C U S T O M -- G R I P P E R
// ============================================================================
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
    //ptr2servo = &GripperServo;

    ptr2servo->attach(_SERVO_PWM_PIN);

    ptr2servo->write(CLOSE_GRIPPER_POSITION);

    //ptr2servo->detach();

    _gripper_state = GRIPPER_CLOSED;

    *gripper_current_state =  _gripper_state;
}

void gripper::closeGripperForce(Servo * ptr2servo, unsigned long grasp_force_limit, tools::gripper_states * gripper_current_state)
{
    unsigned long returned_grasp_force;

    //ptr2servo = &GripperServo;

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
        
        delay(GRIPPER_MOV_DEL_MILLIS);

    } while ( (!object_gripped) && (!impossible_grip) );
    
    //ptr2servo->detach();

    _gripper_state = GRIPPER_CLOSED;

    *gripper_current_state =  _gripper_state;
}

unsigned long gripper::measureForce()
{
    // Returns long value in [N]
    unsigned long force_measured;

    analogReadResolution(DUE_MAX_BITS_RESOL); // ONLY FOR DUE! - > FOR MEGA COMMENT OUT

    _FSR_READING = analogRead(_FSR_AI_PIN); 

    _FSR_VOLTAGE = map(_FSR_READING, 0, DUE_MAX_ANAL_RESOLUTION, 0, DUE_mV);   // FOR MEGA: DUE_ANAL_RESOLUTION -> MEGA_ANAL_RESOLUTION, _FSR_VCC_mV ->DUE_mV

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
    // [23-3-21] added DUE analog specs

    analogReadResolution(DUE_MAX_BITS_RESOL); // ONLY FOR DUE! - > FOR MEGA COMMENT OUT
    
    _FSR_READING = analogRead(_FSR_AI_PIN);

    _FSR_VOLTAGE = map(_FSR_READING, 0, DUE_MAX_ANAL_RESOLUTION, 0, DUE_mV);   // FOR MEGA: DUE_MAX_BITS_RESOL -> MEGA_ANAL_RESOLUTION, _FSR_VCC_mV ->DUE_mV
    //_FSR_VOLTAGE = (int) (_FSR_READING * (DUE_mV / DUE_MAX_ANAL_RESOLUTION) );

    _FSR_CALIBR_OFFSET = _FSR_VOLTAGE;

    int calibrationFsrVoltage = _FSR_VOLTAGE;

    return calibrationFsrVoltage;
}

/*
void gripper::readGripperStateEEPROM(tools::gripper_states * gripper_current_state)
{
    // Reads gripper state from EEPROM - Executed @ setup


   EEPROM.get(CS_GRIPPER_EEPROM_ADDR, *gripper_current_state);

   _gripper_state = (* gripper_current_state);

}

void gripper::writeGripperStateEEPROM(tools::gripper_states * gripper_current_state)
{
    // Writes gripper state from EEPROM - Executed @setup if user wants

   EEPROM.put(CS_GRIPPER_EEPROM_ADDR, *gripper_current_state);
}
*/
// ============================================================================
//  D A T A -- L O G G E R
// ============================================================================
dataLogger::dataLogger(): String(), File(), SDClass()
{
    pinMode(SD_CARD_CS_PIN,OUTPUT);

    _file_response       = false;
    _boolean_sd_response = false;
}

void dataLogger::setupDataLogger(File *ptr2root, debug_error_type * debug_error)
{
   //Serial.println("STARTED setupDataLogger");

   bool initialized_sd = false;
   int total_time;
   unsigned long started_sd_initialization = millis();
   do
   {
        initialized_sd = SD.begin(SD_CARD_CS_PIN);
        if (!initialized_sd)
        {
           //Serial.println("den anoigei");
           *debug_error = SD_INIT_FAILED;
        }
        
        total_time = millis() - started_sd_initialization;
   } while( (!initialized_sd) && (total_time < SD_INIT_TIMEOUT_MILLIS) );

    if (!initialized_sd)
    {
        *debug_error = SD_INIT_FAILED;
    }

    if (initialized_sd)
    {
        *ptr2root = SD.open("/");
        //Serial.println("OPENED ROOT /");
        *debug_error = NO_ERROR;
    }
    
    return;
}

bool dataLogger::createSessionDir( char * session_dir)
{
    // this is called at startup-sets the char name of directory
    // NAME = "TIME_DATE" = "MIN_HR_DAY_MONTH_YEAR"
    // Assumes that time has been set successfully during setup!
    const char *final_name;
    int MIN,HR,DAY,MONTH;//,YEAR;
    
    // itoa buffers
    char MIN_S[3];
    char HR_S[3];
    char DAY_S[3];
    char MONTH_S[3];
    //char * YEAR_S;
    // itoa conversion -> gamietai
    MIN   = minute();   itoa(MIN, MIN_S, 10 );
    HR    = hour();     itoa(HR, HR_S, 10 );
    DAY   = day();      itoa(DAY, DAY_S, 10 );
    MONTH = month();    itoa(MONTH, MONTH_S, 10 );
    //YEAR  = year();     YEAR_S  = itoa(YEAR, conv_buffer, DEC );

    // toCharArray buffers
    //String str;                     // just for string->char
    //char MIN_S[2];
    //char HR_S[2];
    //char DAY_S[2];
    //char MONTH_S[2];
    //char YEAR_S[2];

    // int - String - toCharArray conversion
    //MIN   = minute(); str = String(MIN); str.toCharArray(MIN_S,2);
    //HR    = hour();   str = String(HR);  str.toCharArray(HR_S,2);
    //DAY   = day();    str = String(DAY); str.toCharArray(DAY_S,2);

    // build the dir name "TIME_DATE" described above from ints
    //*session_dir = MIN_S + HR_S + DAY_S + string_spacer + MONTH_S;// + sting_spacer + YEAR_S;
    strcpy(session_dir, MIN_S);
    //strcpy(session_dir, "GAM");
    //strcat(session_dir, "/logs");
    //strcat(session_dir, HR_S);
    //strcat(session_dir, &string_spacer);
    strcat(session_dir, DAY_S);
    strcat(session_dir,"_");
    strcat(session_dir, MONTH_S);

    // create the dir
    final_name = session_dir;
    _boolean_sd_response = SD.mkdir(final_name);
    if(_boolean_sd_response)
    {
        //DEBUG_SERIAL.println("SUCCESS");
        //delay(SD_STABIL_MILLIS);
        return true;
    }
    else
    {
        //Serial.println("FAILED");
        return false;
    }
}

//bool dataLogger::createSensorDir(sensors::sensors_list sensor_choice, String session_dir, String &final_sensor_dir)
bool dataLogger::createSensorDir(sensors::sensors_list sensor_choice, const char * session_dir, char * final_sensor_dir, debug_error_type * debug_error)
{
    // will create a sensor folder inside the session folder provided
    
    switch (sensor_choice)
    {
        case JOINT_POS:
            strcpy(final_sensor_dir, session_dir);
            strcat(final_sensor_dir, "/PS/");
            *debug_error = NO_ERROR;
            break;
        case JOINT_VEL:
            strcpy(final_sensor_dir, session_dir);
            strcat(final_sensor_dir, "/VL/");
            *debug_error = NO_ERROR;
            break;           
        case FORCE_3AXIS:
            strcpy(final_sensor_dir, session_dir);
            strcat(final_sensor_dir, "/FC/");
            *debug_error = NO_ERROR;
            break;
        case IMU_9AXIS:
            strcpy(final_sensor_dir, session_dir);
            strcat(final_sensor_dir, "/IM/");
            *debug_error = NO_ERROR;
            break;
        case CURRENT_JOINT1:
            strcpy(final_sensor_dir, session_dir);
            strcat(final_sensor_dir, "/CR/");
            *debug_error = NO_ERROR;
            break;    
        default:
            *debug_error = BUILD_SENSOR_PATH_FAILED;
            break;
    }

    if ( *debug_error == NO_ERROR )
    {
        _boolean_sd_response = SD.mkdir(final_sensor_dir);
        if(_boolean_sd_response)
        {
            //delay(SD_STABIL_MILLIS);
            return true;
        }
        else
        {
            *debug_error = BUILD_SENSOR_DIR_FAILED;
            return false;
        }
    }
    else
    {
            return false;
     }
    
}

//void dataLogger::createFile(File *ptr2file, String final_sensor_dir,  String &filename , byte OPERATION,  debug_error_type * debug_error)
//void dataLogger::createFile(String final_sensor_dir,  String &filename ,  debug_error_type * debug_error)
void dataLogger::createFile(File *ptr2file, const char * path2file, char * filename ,  debug_error_type * debug_error)
{
    // re-written with no Strings
    const char *final_name;

    int MIN,SEC;

    // itoa buffers
    char MIN_S[3];
    char SEC_S[3];
    // itoa conversion -> gamietai
    SEC = second();   itoa(SEC, SEC_S, 10 );
    MIN = minute();   itoa(MIN, MIN_S, 10 );

    // toCharArray buffers
    //String str;                     // just for string->char
    //char MIN_S[2];
    //char SEC_S[2];
    // String - toCharArray conversion
    //SEC = second(); str = String(SEC);  str.toCharArray(SEC_S,2);
    //MIN = minute(); str = String(MIN);  str.toCharArray(MIN_S,2);

    // name concatenation
    strcpy(filename, path2file);      // First concatenates the global path previously created
    strcat(filename, SEC_S);
    //strcat(filename, &string_spacer);
    strcat(filename, MIN_S);

    strcat(filename, ".txt");
    //strcat(filename, &dot);
    //strcat(filename, &file_ext_log);

    final_name = filename;
    /*
    if (SD.exists(final_name))
    {
        *debug_error = SD_FILE_EXISTS;
    }
    else
    {
        *debug_error = NO_ERROR;
    }
    */
    *debug_error = NO_ERROR;
    if (*debug_error == NO_ERROR)
    {

        //*ptr2file = SD.open(final_name, FILE_WRITE); // since file doesn't exist yet operation MUST BE WRITE, as specified in Ln452 SD.cpp
        *ptr2file = SD.open(final_name, FILE_WRITE);
        if (*ptr2file)
        {
            delay(SD_STABIL_MILLIS);
            //CreatedFile.close();      
            ptr2file->close();          // immediate closes file after creation
        }
        else
        {
            *debug_error = CREATE_FILE_FAILED1;
        }   
    }
    else
    {
        *debug_error = CREATE_FILE_FAILED2;
    }
    
    return;
}

void dataLogger::openFile(File *ptr2file, const char *  filename , uint8_t OPERATION,  debug_error_type * debug_error)
{
    // filename: MUST be the global name assigned after creation! [16-3-21]
    // next try: check the pointer of the file object [19-3-21]


    *ptr2file = SD.open(filename, OPERATION);  // here the file pointer gets the file that points to!
    
    if( *ptr2file )
    {
        *debug_error = NO_ERROR;
    }
    else
    {
        *debug_error = OPEN_FILE_FAILED;
    }
    return;

}

void dataLogger::closeFile(File *ptr2file, debug_error_type * debug_error)
{
    // assumes file was open!!!!
    ptr2file->close();
    *debug_error = NO_ERROR;         // => does nothing
    return;
}

//template <class T>
//void dataLogger::writeData(T data2write, unsigned long timestamp, unsigned long data_cnt, File *ptr2file, debug_error_type * debug_error)
void dataLogger::writeData(float * data2write, int size, unsigned long timestamp, unsigned long data_cnt, File *ptr2file, debug_error_type * debug_error)
{
    // the file object must be opened before write! Writes the current data given from
    // sensor to the specified file. file_state was given a value
    // of NO_ERROR OR OPEN_FILE_FAILED. This function is executed only if NO_ERROR
    // was received!
    // [26-3-21] passing pointer to data values and the size of data elements
    
    if (*debug_error == NO_ERROR)
    {
        ptr2file->print(timestamp,DEC);  // first write the time
        
        // DATA_VAL(s)
        for (size_t i = 0; i < size; i++)
        {
            ptr2file->print(" , "); ptr2file->print(data2write[i],DEC);  // write the data in same row
        }

        ptr2file->print(" , "); ptr2file->println(data_cnt,DEC);  // final write cnt and chnage line for next writing

    }
    else
    {
        *debug_error = DATA_WRITE_FAILED;
    }
    
    return;
}

void dataLogger::writeData2LogArray(float * data2write, int size, unsigned long timestamp, byte data_cnt, real_time_log_struct * ptr2log_struct, debug_error_type * debug_error)
{
    // this will be used inside execute5 function but now data will be saved to matrix of predetermined 
    // max rows instead of sd file.
    
    if (data_cnt < MAX_LOG_DATA )
    {
        // assign time 
        ptr2log_struct->timestamps[data_cnt] = timestamp;
        // assign counter
        ptr2log_struct->timestamps[data_cnt] = data_cnt;
        // assign the logged data
        for (size_t i = 0; i < size; i++)
        {
            ptr2log_struct->logged_data[data_cnt][i] = data2write[i];
        }
    }
    else
    {
        *debug_error = DATA_WRITE_FAILED;
    }
    
    return;
}

void dataLogger::finalWriteData( real_time_log_struct * ptr2log_struct, File *ptr2file, debug_error_type * debug_error)
{
    // the file object must be opened/closed before/after write! The diff here is that all
    // values saved in logged_data during task execution are saved in the file
    // and then cleared. logged_data is a 2D array log_data_max x nDof !
    // [9-4-21] because sd card is playing tough!
    
    if (*debug_error == NO_ERROR)
    {
        for (size_t i = 0; i < MAX_LOG_DATA; i++)
        {
            ptr2file->print(ptr2log_struct->timestamps[i],DEC);  // first write the time
            
            // DATA_VAL(s)
            for (size_t j = 0; j < TOTAL_ACTIVE_JOINTS; j++)
            {
                ptr2file->print(" , "); ptr2file->print(ptr2log_struct->logged_data[i][j],DEC);  // write the data in same row
            }

            ptr2file->print(" , "); ptr2file->println(ptr2log_struct->data_cnts[i],DEC);  // final write cnt and chnage line for next writing
        }
    }
    else
    {
        *debug_error = DATA_WRITE_FAILED;
    }
    
    return;
}

void dataLogger::readData(char * data_buffer, int & buffer_size, int max_data_length, File *ptr2file, debug_error_type * debug_error)
{
    // the file object must be opened before read! 
    
    if (*ptr2file)
    {
        while(*ptr2file)
        {
            buffer_size = ptr2file->readBytes(data_buffer, max_data_length);
        }
    }
    else
    {
        *debug_error = FILE_NOT_FOUND;
    }
    
    return;
}