/* include the necessary libraries for the control algorithm */

#include <Arduino.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"                            
#endif

/* set up the MPU6050 */

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 mpu;                              
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

/* set up the servos */

Servo rollServo;
Servo pitchServo;

#define rollServoPin 3
#define pitchServoPin 4

// servo vars
const int intial_rollServoVal = 82;
const int intial_pitchServoVal = 100;
int rollServoVal = 82;
int pitchServoVal = 100;

/* set up pid control algorithm vars */

float rollActual, pitchActual;
double dT, previousTime, currentTime;
double rollError, rollErrorPrevious;
double rollTarget = 0;
double rollKp = 0.15, rollKi = 0.00001, rollKd = 9; 
double rollProportional=0, rollIntegral=0, rollDerivative=0;
double rollOutput=0;

double pitchError, pitchErrorPrevious;
double pitchTarget = 0;
double pitchKp = 0.15, pitchKi =0.00001, pitchKd = 9; 
double pitchProportional=0, pitchIntegral=0, pitchDerivative=0;
double pitchOutput=0;

/* set up the communication LEDs */

#define LED_PIN_1 5 
#define LED_PIN_2 6

/* set up the button for payload insert */

#define button 12
bool status = false;
bool buttonState = false;
bool lastButtonState = false;
bool pidPaused = false;             // Variable to track whether PID is paused
bool systemActive = true;           // Variable to track the system's active state
bool servoMoved = false;            // Variable to track whether servo has been moved
int insert_angle = 50;
int buttonPress = 0;
int pos;

/* declare functions */

void blink(int LED);
void pid();
void servo_control();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // configure LED for output
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);

    // turn on blue LED to indicate initial set up
    digitalWrite(LED_PIN_1, HIGH);

    // configure servos for outpit
    rollServo.attach(rollServoPin);
    pitchServo.attach(pitchServoPin);
    delay(100); 
    pitchServo.write(pitchServoVal);
    rollServo.write(rollServoVal);
    delay(100);

    // configure button for input
    pinMode(button, INPUT_PULLUP);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.setWireTimeout(3000, true); //timeout value in uSec
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    //mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(22);
    mpu.setZGyroOffset(91);
    mpu.setZAccelOffset(1272); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // turn off the blue LED to indicate initial set up complete
    digitalWrite(LED_PIN_1, LOW);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // Read the current state of the button
    buttonState = digitalRead(button);

    // Check if the button state has changed from high to low
    if (buttonState == LOW && lastButtonState == HIGH) {
        // Toggle the PID paused state
        pidPaused = !pidPaused;
        // add the button state change to a counter
        buttonPress++;
        
    // If PID is paused, move the servo to insert position
    if (pidPaused && !servoMoved && buttonPress == 1) {
        // turn on blue LED to indicate payload insert mode
        digitalWrite(LED_PIN_1, HIGH);
        // move servo in a sweep action to insert payload
        for (pos = intial_pitchServoVal; pos>= intial_pitchServoVal - insert_angle; pos-= 1) {
            pitchServo.write(pos);
            delay(15);
            }

        servoMoved = true;
        }
    }
    
    // Check if the servo has been moved and 20 seconds have passed
    if (servoMoved && buttonPress == 2) {
        // Return the servo to its original position
        for (pos = intial_pitchServoVal - insert_angle; pos <= intial_pitchServoVal; pos += 1) {
              pitchServo.write(pos);
              delay(15);
             }
        rollServo.write(intial_rollServoVal);
        servoMoved = false;
        buttonPress = 0;
        delay(100);
    }

    // Store the current button state for comparison in the next iteration
    lastButtonState = buttonState;

    // Check if the system is active before proceeding
    if (systemActive && !pidPaused) { 
    // Check DEVSTATUS and control LED accordingly
    if (devStatus == 1 || devStatus == 2 || !dmpReady ) {
      digitalWrite(LED_PIN_2, LOW);
      blink(LED_PIN_1);
    }
    else {
        digitalWrite(LED_PIN_2, HIGH);
        digitalWrite(LED_PIN_1, LOW);    
    }
    
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            rollActual = ypr[2] * 180 / M_PI;
            pitchActual = ypr[1] * 180 / M_PI;

            // execute PID loop
            pid();

            // Update servo position based on PID output
            servo_control();
            
            Serial.println(pitchActual);
        #endif
        }
    }
}

void pid() {
 pitchError = pitchTarget - pitchActual;
 rollError = rollTarget - rollActual;

 currentTime = millis();
 dT = (currentTime - previousTime);

 pitchProportional = pitchError;
 rollProportional = rollError;

 pitchIntegral += dT * pitchError;
 rollIntegral += dT * rollError;

 pitchDerivative = (pitchError - pitchErrorPrevious) / dT;
 rollDerivative = (rollError - rollErrorPrevious)/ dT;

 pitchOutput = (pitchKp * pitchProportional) + (pitchKi * pitchIntegral) + (pitchKd * pitchDerivative);
 rollOutput = (rollKp * rollProportional) + (rollKi * rollIntegral) + (rollKd * rollDerivative);

 previousTime = currentTime;
 pitchErrorPrevious = pitchError;
 rollErrorPrevious = rollError;
}

void servo_control() {
  if (abs(pitchError) > 0) {
    pitchServoVal -= pitchOutput;
    pitchServo.write(pitchServoVal);
    pitchServoVal = constrain(pitchServoVal, intial_pitchServoVal - 50, intial_pitchServoVal + 50);
  }
  if (abs(rollError) > 0) {
    rollServoVal -= rollOutput;
    rollServo.write(rollServoVal);
    rollServoVal = constrain(rollServoVal, intial_rollServoVal - 50, intial_rollServoVal + 50);
  }
}

void blink(int LED) {
  digitalWrite(LED, HIGH); 
  delay(200);              
  digitalWrite(LED, LOW);   
  delay(200);                     
}