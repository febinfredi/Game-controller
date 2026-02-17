#include <SPI.h>
#include <Arduino.h>
#include <SimpleFOC.h>


#ifdef IS_LEONARDO
#elif IS_NANO
  // Pin definitions
  // MT6835 SPI pins
  const int MT6835_CSN_PIN = 10;
  // Motor driver pins
  const int EN_PIN = 8;
  const int IN1_PIN = 3;
  const int IN2_PIN = 6;
  const int IN3_PIN = 9;
  
  BLDCMotor motor = BLDCMotor(11); // (pole pair number, phase resistance (optional));
  BLDCDriver3PWM driver = BLDCDriver3PWM(IN1_PIN, IN2_PIN, IN3_PIN, EN_PIN); // (phA, phB, phC, enable)

  // calibration variables
  float min_angle = 0;
  float max_angle = 0;
  int zone = 0;
#endif


// flag to indicate calibrate mode
int calibration_mode = 1;


// function to get angle
float getAngle() {
    uint16_t command = (0x3 << 12) | (0x003 & 0xFFF);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    digitalWrite(MT6835_CSN_PIN, LOW);

    SPI.transfer16(command);
    uint8_t data0 = SPI.transfer(0x00);
    uint8_t data1 = SPI.transfer(0x00);
    uint8_t data2 = SPI.transfer(0x00);
    uint8_t data3 = SPI.transfer(0x00);

    digitalWrite(MT6835_CSN_PIN, HIGH);

    SPI.endTransaction();

    uint32_t data = ((uint32_t)data0 << 13) | (data1 << 5) | (data2 >> 3);

    float angle_deg = (double)data / 2097152.0 * 360.0;

    // Check warnings
    uint8_t status = data2 & 0x7;
    if (status & 0x01) {
        Serial.println("Warning : overspeed!");
        angle_deg = -1; // invalid angle
    }
    if (status & 0x02) {
        Serial.println("Warning : Low magnetic field!");
        angle_deg = -1; // invalid angle
    }
    if (status & 0x04) {
        Serial.println("Warning : Low voltage!");
        angle_deg = -1; // invalid angle
    }

    return angle_deg;
}


float readMySensorCallback(){
    // read my sensor
    // return the angle value in radians in between 0 and 2PI
    double angle_deg = getAngle();
    return (float)(angle_deg * M_PI / 180.0);
}


char getChar() {
    while (Serial.available() == 0) {}
    char key = Serial.read();            
    _delay(10); 
    return key;
}


// function to calibrate the motor movement range
void calibrateMotor(){
    Serial.println("Starting calibration...");
    
    // get min angle
    Serial.println("Rotate to min angle and press 'y' to confirm or any other key to display the current min angle");
    while (getChar() != 'y') {
        Serial.println("Min angle read: " + String(readMySensorCallback()) + " rad");
        Serial.println("Press 'y' to confirm the min angle \n");
    }
    min_angle = readMySensorCallback();
    Serial.println("Min angle recorded: " + String(min_angle) + " rad\n");

    // get max angle
    Serial.println("Rotate to max angle and press 'y' to confirm or any other key to display the current max angle");
    while (getChar() != 'y') {
        Serial.println("Max angle read: " + String(readMySensorCallback()) + " rad");
        Serial.println("Press 'y' to confirm the max angle \n");
    }
    max_angle = readMySensorCallback();
    Serial.println("Max angle recorded: " + String(max_angle) + " rad\n");
}


// GenericSensor class constructor
//  - readCallback pointer to the function reading the sensor angle
//  - initCallback pointer to the function initialising the sensor (optional)
GenericSensor MT6835 = GenericSensor(readMySensorCallback);


// timestamp for changing direction
unsigned long timestamp_us = _micros();

float target_angle = 0.0;


void setup() {
    // Serial communication initialisation
    Serial.begin(115200);


    // FOR MT6835 SENSOR INITIALIZATION
    // Configuration of the CSN pin
    pinMode(MT6835_CSN_PIN, OUTPUT);
    // SPI communication initialisation with the MT6835 parameters
    SPI.begin();

    //------------------------------------------------------------

    // FOR MOTOR AND ENCODER INITIALIZATION
    // initialize encoder
    MT6835.init();

    // link motor and the encoder
    motor.linkSensor(&MT6835);

    // driver config
    driver.voltage_power_supply = 24;
    driver.voltage_limit = 6;
    if(!driver.init()){
      Serial.println("Driver init failed!");
      return;
    }

    // link the motor and the driver
    motor.linkDriver(&driver);

    // set motion control loop to be used
    motor.controller = MotionControlType::angle;
 
    // velocity PID controller parameters
    // default P=0.5 I = 10 D =0
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 20;
    motor.PID_velocity.D = 0.001;
    // jerk control using voltage voltage ramp
    // default value is 300 volts per sec  ~ 0.3V per millisecond
    motor.PID_velocity.output_ramp = 1000;

    // velocity low pass filtering
    // default 5ms 
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.01;

    // angle P controller -  default P=20
    motor.P_angle.P = 20;

    //  maximal velocity of the position control
    // default 20
    motor.velocity_limit = 4;

    // limiting motor movements
    // limit the voltage to be set to the motor
    // current = voltage / resistance, so try to be well under 1Amp
    motor.voltage_limit = 12;   // [V]

    // init motor hardware
    if(!motor.init()){
      Serial.println("Motor init failed!");
      return;
    }

    // align encoder and start FOC
    motor.initFOC();

    Serial.println("Motor ready!");
    _delay(1000);

    // if calibration mode flag is high, start calibration
    if (calibration_mode) {
        calibrateMotor();
    }
}


void loop() {

	// main FOC algorithm function
    motor.loopFOC();

    motor.move(target_angle);

    if (_micros() - timestamp_us > 5e5) {

        if (target_angle == min_angle){
            target_angle = max_angle;
        }
        else if (target_angle == max_angle){
            target_angle = min_angle;
        }
        else { 
            target_angle = (min_angle + max_angle)/2.0;
        }

        Serial.println("Min angle: " + String(min_angle));
        Serial.println("Max angle: " + String(max_angle));
        Serial.println("Target angle: " + String(target_angle) + "\n");

        timestamp_us = _micros();
    }
  

}

