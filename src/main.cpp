#include <SPI.h>
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>
#include <SimpleFOCDrivers.h>                      
#include "encoders/mt6835/MagneticSensorMT6835.h"

// Reset SimpleFOC sensor members (int32_t Sensor::full_rotations and float Sensor::angle_prev)
class SensorInspector : public Sensor {
public:
    void forceReset() {
        full_rotations = 0;
        angle_prev = 0;
    }
};

// Pin definitions for ESP32
const int MT6835_CSN_PIN = 5;
const int EN_PIN = 33;
const int IN1_PIN = 25;
const int IN2_PIN = 26;
const int IN3_PIN = 27;

// variables for setting min and max range of lever motion
float min_angle = 0.0;  // joystick min angle - for calibration 
float max_angle = 0.0;  // joystick max angle - for calibration
int zone = 0;
float err = 0.0;    // error = target - current_val
float tq_out = 0.0;     // output torque voltage  
float target_angle = 0.00;  
float zero_elec_angle = 10.0;   // variable to store motor.zero_electric_angle value
int sensor_direction = 0;   // variable to store motor.sensor_direction value
bool calib_done = false;    // flag to check if calibration has been performed once
int ctr_calib = 0;  // counter for calibration
bool calibration_mode = 0;  // flag to enter calibrate mode

// parameters for spring torque     
const float SNAP_STRENGTH = 12.0;
const float SPRING_DAMPING_FACTOR = 0.002; 

// parameters for friction torque
const float CONSTANT_DRAG = 2.0; 
const float FADE_START_VELOCITY = 1.2;  // velocity where drag starts scaling down 
const float DEADZONE = 0.1; 
const float FRICTION_DAMPING_FACTOR = 0.04;


// Use GPIO 18 (SCK), 19 (MISO), 23 (MOSI) by default
BLDCMotor motor = BLDCMotor(11, 9.8, 52.8); 
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1_PIN, IN2_PIN, IN3_PIN, EN_PIN);

// create an instance of preferences library
Preferences prefs;

char getChar() {
    while (Serial.available() == 0) {}
    char key = Serial.read();            
    _delay(10); 
    return key;
}


void calibrate(Sensor& sens, BLDCMotor& mot, Preferences& pref){

    // calibrate Motor min and max angle
    Serial.println("Starting calibration...");

    min_angle = 0.0;
    max_angle = 0.0;

    // get min angle
    Serial.println("Rotate to min angle and press 'y' to confirm and then same step for max angle and press any other key for reading current angle");
    while (ctr_calib!=2){
        motor.loopFOC();
        if (Serial.available() > 0){
            char incoming_char = Serial.read();
            if (incoming_char != '\n' && incoming_char != '\r') {
                if (incoming_char == 'y' && ctr_calib==0) {
                    Serial.println("Min angle read: " + String(sens.getAngle()) + " rad");
                    min_angle = sens.getAngle();
                    ctr_calib+=1;
                }
                else if (incoming_char == 'y' && ctr_calib==1) {
                    Serial.println("Max angle read: " + String(sens.getAngle()) + " rad");
                    max_angle = sens.getAngle();
                    ctr_calib+=1;
                }
                else {
                    Serial.println("Current Angle read: " + String(sens.getAngle()) + " rad");
                }   
            }
        }
    }    

    if (ctr_calib==2) {
        // preferences R/W mode
        Serial.println("Writing min and max angle to memory...");
        pref.begin("motor_params", false);
        pref.putBool("calib_done", true);
        pref.putFloat("min_angle", min_angle);
        pref.putFloat("max_angle", max_angle);
        pref.end();
    }
    
    mot.loopFOC();
}


float spring_tq(Sensor& sens, BLDCMotor& mot, float target){
    // set controller type 
    mot.torque_controller = TorqueControlType::voltage;
    // return calculated torque
    // Serial.println(1*((SNAP_STRENGTH*(target - sensor.getAngle())) - (SPRING_DAMPING_FACTOR*(motor.shaft_velocity))));
    return -1*((SNAP_STRENGTH*(target - (sens.getAngle()-min_angle))) - (SPRING_DAMPING_FACTOR*(motor.shaft_velocity)));
}


float friction_tq(Sensor& sens, BLDCMotor& mot){
    // set controller type 
    mot.controller = MotionControlType::torque;
    mot.torque_controller = TorqueControlType::voltage;

    float damp = -1 * FRICTION_DAMPING_FACTOR * mot.shaft_velocity;

    if (abs(mot.shaft_velocity) > FADE_START_VELOCITY) {
        // drag opposing the direction
        if (mot.shaft_velocity > 0) return -CONSTANT_DRAG + damp;
        else return CONSTANT_DRAG + damp;
    } 
    else if (abs(mot.shaft_velocity) > DEADZONE) {
        // smoothly transition drag from max to 0
        float fade_factor = (abs(mot.shaft_velocity) - DEADZONE) / (FADE_START_VELOCITY - DEADZONE);
        if (mot.shaft_velocity > 0) return (-CONSTANT_DRAG * fade_factor) + damp;
        else return (CONSTANT_DRAG * fade_factor) + damp;
    } 
    else {
        // zero drag when completely stopped
        return 0.0;
    }
}


// Instantiate the specialized MT6835 sensor class natively
MagneticSensorMT6835 sensor = MagneticSensorMT6835(MT6835_CSN_PIN);


void setup() {
    // Serial communication initialisation
    Serial.begin(115200);

    // FOR MT6835 SENSOR INITIALIZATION
    // Configuration of the CSN pin
    pinMode(MT6835_CSN_PIN, OUTPUT);
    digitalWrite(MT6835_CSN_PIN, HIGH);

    // SPI communication initialisation with the MT6835 parameters
    SPI.begin();

    // preferences R/W mode
    prefs.begin("motor_params", false);
    sensor_direction = prefs.getInt("sens_dir", 0);
    zero_elec_angle = prefs.getFloat("zero_elec_ang", 10.00);
    min_angle = prefs.getFloat("min_angle", 0.0);
    max_angle = prefs.getFloat("max_angle", 0.0);
    calib_done = prefs.getBool("calib_done", false);
    prefs.end();


    // FOR MOTOR AND ENCODER INITIALIZATION
    // initialize encoder
    sensor.init();

    sensor.update();
    // clear the turn counter variables out of memory
    ((SensorInspector*)&sensor)->forceReset();
    sensor.update();

    // link motor and the encoder
    motor.linkSensor(&sensor);

    // driver config
    driver.voltage_power_supply = 24;
    driver.voltage_limit = 12;
    if(!driver.init()){
      Serial.println("Driver init failed!");
      return;
    }

    // link the motor and the driver
    motor.linkDriver(&driver);

    // limit the voltage to be set to the motor
    motor.voltage_limit = 12;

    // init motor hardware
    if(!motor.init()){
      Serial.println("Motor init failed!");
      return;
    }
    
    // align encoder and start FOC
    if (sensor_direction==-1 && zero_elec_angle!=10.0){
        Serial.println("Valid encoder align params");
        motor.sensor_direction = Direction::CCW;
        motor.zero_electric_angle = zero_elec_angle;
    }
    else if (sensor_direction==1 && zero_elec_angle!=10.0){
        Serial.println("Valid encoder align params");
        motor.sensor_direction = Direction::CW;
        motor.zero_electric_angle = zero_elec_angle;
    }
    else {
        Serial.println("Invalid encoder align params: Init encoder align and start FOC");
        if(!motor.initFOC()){
            // if motor init fails
            Serial.println("Motor Init Failed: DISABLING MOTOR DRIVER");
            motor.disable();
            while(true){delay(1000);}

        }
        else{
        prefs.begin("motor_params", false);
        prefs.putInt("sens_dir", motor.sensor_direction);
        prefs.putFloat("zero_elec_ang", motor.zero_electric_angle);
        prefs.end();
        }
    }

    motor.loopFOC();  // run once to get a fresh shaft_angle reading
    
    // if calibration mode flag is high, enter calibration mode
    if (calibration_mode || !calib_done) {
        // calibrate Motor min and max angle
        calibrate(sensor, motor, prefs);        
    }

    Serial.println("SETUP COMPLETE");

    _delay(1000);

}



void tq_zone_profile(float minAngle, float maxAngle){

}
 


void loop() {
    
    motor.loopFOC();

    
    
    Serial.print(min_angle);
    Serial.print(" | ");
    Serial.print(max_angle);
    Serial.print(" | "); 
    Serial.print(sensor.getAngle()-min_angle);
    // Serial.print(" | ");
    // Serial.print(calib_done);
    Serial.println();
    
    // motor.move(spring_tq(0.00));


}