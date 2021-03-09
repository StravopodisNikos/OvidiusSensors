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
    // modified to execute for all calls

    ptr2hx711 = & ForceSensorAxis;

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

void force3axis::getCurrentState(HX711 * ptr2hx711, sensors::force_sensor_states * force_current_state, debug_error_type * debug_error)
{
    ptr2hx711 = & ForceSensorAxis;

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

// ============================================================================
//  D A T A -- L O G G E R
// ============================================================================
dataLogger::dataLogger();

bool dataLogger::createSessionDir()
{
    // this is called at startup-sets the char name of directory
    // NAME = "TIME_DATE" = "MIN_HR_DAY_MONTH_YEAR"
    // Assumes that time has been set successfully during setup!
    int MIN,HR,DAY,MONTH,YEAR;
    String MIN_S,HR_S,DAY_S,MONTH_S,YEAR_S;

    String BUILT_DIR_NAME = String();
    String sting_spacer   = String("_");
    // first receive int of MIN-HR-DAY-MONTH-YEAR
    MIN   = minute();   MIN_S   = String(MIN,DEC);
    HR    = hour();     HR_S    = String(HR,DEC);
    DAY   = day();      DAY_S   = String(DAY,DEC);
    MONTH = month();    MONTH_S = String(MONTH,DEC);
    YEAR  = year();     YEAR_S  = String(YEAR,DEC);

    // build the dir name "TIME_DATE" described above from ints
    BUILT_DIR_NAME = MIN_S + sting_spacer + HR_S + sting_spacer + DAY_S + sting_spacer + MONTH_S + sting_spacer + YEAR_S;

    // create the dir
    if(SD.mkdir(BUILT_DIR_NAME))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool dataLogger::createSensorDir(sensors::sensors_list sensor_choice, String *session_dir, String &final_sensor_dir)
{
    // will create a sensor folder inside the session folder provided
    String expl_sensor_dir  = String();
    String folder_splitter  = String("/");

    switch (sensor_choice)
    {
        case FORCE_3AXIS:
            expl_sensor_dir = String("FORCE");
            break;
        case IMU_9AXIS:
            expl_sensor_dir = String("IMU");
            break;
        case CURRENT_JOINT1:
            expl_sensor_dir = String("CURRENT");
            break;    
        default:
            return false;
            break;
    }

    final_sensor_dir = *session_dir + folder_splitter + expl_sensor_dir;

    if(SD.mkdir(final_sensor_dir))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void dataLogger::setupDataLogger(debug_error_type * debug_error)
{
   bool initialized_sd = false;
   int total_time;
   unsigned long started_sd_initialization = millis();
   do
   {
        initialized_sd = SD.begin(SD_CARD_CS_PIN);

        total_time = millis() - started_sd_initialization;
   } while( (!initialized_sd) && (total_time < SD_INIT_TIMEOUT_MILLIS) );

    if (!initialized_sd)
    {
        *debug_error = SD_INIT_FAILED;
    }
    
    return;
}

void dataLogger::openFile(File *ptr2file, char *filename , byte OPERATION,  debug_error_type * debug_error)
{
    // File must open at start of corresponding ino file!
    // OPERATION -> FILE_READ OR FILE_WRITE

    *ptr2file = SD.open(filename, OPERATION);

    if (*ptr2file)
    {
        *debug_error = NO_ERROR;
    }
    else
    {
        *debug_error = OPEN_FILE_FAILED;
    }
    
    return;
}

void dataLogger::closeFile(File *ptr2file)
{
    // File must close at start of corresponding ino file!
    ptr2file->close();
}

template <class T>
void dataLogger::writeData(T data2write, unsigned long timestamp, unsigned long &data_cnt, File *ptr2file, debug_error_type * debug_error)
{
    // the file object must be opened before write! file_state was given a value
    // of NO_ERROR OR OPEN_FILE_FAILED. This function is executed only if NO_ERROR
    // was received!
    
     if (*ptr2file)
     {
         // CNT, TIMESTAMP_MILLIS, DATA_VAL
        ptr2file->print(data_cnt,DEC); ptr2file->print(" , "); ptr2file->print(timestamp,DEC); ptr2file->print(" , "); ptr2file->println(data2write,DEC);
        *debug_error = NO_ERROR;
     }
     else
     {
        *debug_error = DATA_WRITE_FAILED;
     }
      
    return;
}