#include <Arduino.h>
#include <Wire.h>
#include <TinyMPU6050.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#include "RES_Smoothing_Data.h"
#include "SimpleKalmanFilter.h"
#include "SimpleDeadReckoning.h"

/*
 * Updated for using the arduino nano as command
 */

// Todo redefine the pins

/*
 * TODO for next milestone
 * Draw a square and make the robot go places inside the square // WORK ON THIS NEXT
 * Make the nano figure out where it is on a xy axis. Drawing is done by the mega on the controller
 * Also show the current location in that square via the OLED display
 * The OLED is 128 x 64 - use the entire screen as the "map"
 * Save the progress of the robot as a line in the map
 * Set up the nano (that will be the robot) bluetooth receiver as slave
 * And pair with the mega (that will be the controller)
 * See how that is made so they search for eachother
 *
 */

/*
 * Serial message definitions
 */

#define ATR_PROTOCOL_MAX_BUF_SIZE   64

char msgBuf[ATR_PROTOCOL_MAX_BUF_SIZE];
int msgBufPnt = 0;

/*
 * Timekeeping
**/
long cTime = 0;     // current time of each iteration

/*
 * Joystick variable saving
 */
int joystick[3] = {0,0,0};

#define SPACE "   "

/*
 * Encoder/Motor variables
 */

#define PIN_WHEEL_ENCODER_CLK_RIGHT    3
#define PIN_WHEEL_ENCODER_DT_RIGHT    5
#define PIN_MOTOR_PWM_RIGHT    11
#define PIN_MOTOR_IN0_RIGHT    A0
#define PIN_MOTOR_IN1_RIGHT    9

#define PIN_WHEEL_ENCODER_CLK_LEFT    2
#define PIN_WHEEL_ENCODER_DT_LEFT    4
#define PIN_MOTOR_PWM_LEFT    6
#define PIN_MOTOR_IN0_LEFT    7
#define PIN_MOTOR_IN1_LEFT    8

int prevLeftCLK, prevLeftDT, nowLeftCLK, nowLeftDT = 0;
int prevRightCLK, prevRightDT, nowRightCLK, nowRightDT = 0;

//lCounter increases going forward, while rCounter decreases going forward
long lCounter, rCounter = 0;
long lastlCounter, lastrCounter = 0;
float cTheta, xLocation, yLocation = 0;

SimpleDeadReckoning mySDR( 104.0, 3.39, 13.0, 0);   // encoder values per one rotation,  wheel radius, distance between two wheels, unit (cm)
float thetaOffset = 0.0;

double pwmSpeed[2] = {1.0, 1.0};
double realSpeed[2] = {0.0, 0.0};

/*
 * Scanner pins
 */

#define PIN_SONAR_PING              A3
#define PIN_SONAR_ECHO              A2
#define PIN_SERVO                   10

#define MIN_SONAR_ANGLE      30
#define MID_SONAR_ANGLE      90
#define MAX_SONAR_ANGLE      150
#define MAX_SONAR_VALUE       350      // e.g. 650 mm is max range
#define STRID_SONAR_ANGLE     5

// Scan variables

const int numReadings = 10;

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// create servo object to control a servo
Servo myServo;

int readings[numReadings];      // the readings from the input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int sonar[2][180];
int sonarAngle  = 30;
int sonarScanDirection = 0;
float prev = 0.0; float now = 0.0; float alpha = 0.99;  float beta = 0.01;

/*
 * PID
 */
double pidSetpoint, pidInput, pidOutput;
double Kp = 2, Ki = 5, Kd = 1;

PID pidR(&realSpeed[0], &pwmSpeed[0], &pidSetpoint, Kp, Ki, Kd, DIRECT);
PID pidL(&realSpeed[1], &pwmSpeed[1], &pidSetpoint, Kp, Ki, Kd, DIRECT);

/*
 * MPU
 */

MPU6050 mpu(Wire);

/*
 * Function declarations
 */

void handleScan();

void handleJoystick(int xValue, int yValue);

void handleSerialComm();

void handleLocalization();

bool checkMessage();

void checkLeftEncoder();

void checkRightEncoder();

void handleAsync();

/*
 * Main Arduino functions
 */

SoftwareSerial sSerial = SoftwareSerial(13, 12);
void setup() {
    Serial.begin(9600);
    sSerial.begin(9600);

    // Wheel Encoder Setting
    pinMode(PIN_WHEEL_ENCODER_CLK_LEFT, INPUT_PULLUP);
    pinMode(PIN_WHEEL_ENCODER_CLK_RIGHT, INPUT_PULLUP);
    pinMode(PIN_WHEEL_ENCODER_DT_LEFT, INPUT_PULLUP);
    pinMode(PIN_WHEEL_ENCODER_DT_RIGHT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_ENCODER_CLK_LEFT), checkLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_ENCODER_DT_LEFT), checkLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_ENCODER_CLK_RIGHT), checkRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_ENCODER_DT_RIGHT), checkRightEncoder, CHANGE);

    // Set up motor pins - A
    pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
    pinMode(PIN_MOTOR_IN0_RIGHT, OUTPUT);
    pinMode(PIN_MOTOR_IN1_RIGHT, OUTPUT);

    // Set up motor pins - B
    pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
    pinMode(PIN_MOTOR_IN0_LEFT, OUTPUT);
    pinMode(PIN_MOTOR_IN1_LEFT, OUTPUT);

    // IMU init
    //mpu.Initialize();
//    Serial.println("Calibrating MPU");
//    mpu.Calibrate();
//    Serial.println("MPU calibrated");
//    Serial.println("Offsets:");
//    Serial.print("GyroX Offset = ");
//    Serial.println(mpu.GetGyroXOffset());
//    Serial.print("GyroY Offset = ");
//    Serial.println(mpu.GetGyroYOffset());
//    Serial.print("GyroZ Offset = ");
//    Serial.println(mpu.GetGyroZOffset());

    // PID init
    pidR.SetMode(AUTOMATIC);
    pidL.SetMode(AUTOMATIC);

    // Sonar Setting
    pinMode(PIN_SONAR_PING, OUTPUT);
    pinMode(PIN_SONAR_ECHO, INPUT);
    myServo.attach(PIN_SERVO);
    myServo.write(MID_SONAR_ANGLE);// attaches the servo on PIN_SERVO to the servo object and set middle position

    // init for sonar reading
    for (int i = 0; i < 180; i++) {
        sonar[0][i] = 0; sonar[1][i] = 0;
    }
}

void loop() {
    handleAsync();
}

unsigned long oneMillis = millis();
unsigned long hundredMillis = millis();

void handleAsync() {
    if (millis() > oneMillis + 50) {
        oneMillis = millis();

        handleSerialComm();
        handleJoystick(joystick[0], joystick[1]);
        handleScan();
        handleLocalization();
    }

    if (millis() > hundredMillis + 20) {
        hundredMillis = millis();
        checkLeftEncoder();
        checkRightEncoder();
    }
}

void handleScan() {
    // Servo motor position
    if (sonarScanDirection == 0){ // increase scan angle
        sonarAngle++;
        if (sonarAngle >= 150) sonarScanDirection = 1;
    }else{
        sonarAngle--;
        if (sonarAngle <= 30) sonarScanDirection = 0;

    }
    myServo.write(sonarAngle);
    //delay(20);
    // sonar data sending
    digitalWrite(PIN_SONAR_PING, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_SONAR_PING, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_SONAR_PING, LOW);
    int tmpSonarValue = (pulseIn(PIN_SONAR_ECHO, HIGH,5000 ) / 29 / 2) * 5 ;   // mm
    if (tmpSonarValue == 0) tmpSonarValue = MAX_SONAR_VALUE;
    sonar[1][sonarAngle] = sonar[0][sonarAngle];
    sonar[0][sonarAngle] = tmpSonarValue;
    Serial.print("sonar ");
    Serial.print(sonarAngle);
    Serial.print(" ");
    Serial.print(sonar[1][sonarAngle]);
    Serial.print(" ");
    Serial.println(sonar[0][sonarAngle]);

    // TODO perhaps change how the serial comms are made here
}

/**
 * This is the main function that handles the direction we are moving to
 * Checks the joystick variable and moves wherever it is pointing to
 * Joystick values are organized into X, Y and SW
 */
 
int deadzone = 10;
unsigned int xSpeed = 0, ySpeed = 0;

/**
 * This is the function that regulates where we want to go
 * 1 is forward, -1 is backward, basically
 *
 * @param left
 * @param right
 */
void motorRotation(int left, int right) {
    digitalWrite(PIN_MOTOR_IN0_LEFT, left == 1 ? LOW : HIGH);
    digitalWrite(PIN_MOTOR_IN1_LEFT, left == 1 ? HIGH : LOW);
    digitalWrite(PIN_MOTOR_IN0_RIGHT, right == 1 ? LOW : HIGH);
    digitalWrite(PIN_MOTOR_IN1_RIGHT, right == 1 ? HIGH : LOW);
}

void handleJoystick(int xValue, int yValue) {
    if (yValue >= (512 + deadzone))//Forward
    {
        //Serial.println("Forward");
        ySpeed = (yValue - 512) / 2; // 0 - 255
        if(xValue > (512 + deadzone)) //Left
        {
            //Serial.println("Left");
            xSpeed = (xValue - 512) / 2;
            //analogWrite(LMF, ySpeed - xSpeed); analogWrite(RMF, ySpeed);
            motorRotation(1, 1);
            analogWrite(PIN_MOTOR_PWM_LEFT, ySpeed - xSpeed);
            analogWrite(PIN_MOTOR_PWM_RIGHT, ySpeed);
            //digitalWrite(LMR, LOW); digitalWrite(RMR, LOW);
        }
        else if (xValue < (512 - deadzone)) //Right
        {
            //Serial.println("Right");
            xSpeed = (512 - xValue) / 2;
            //analogWrite(LMF, ySpeed); analogWrite(RMF, ySpeed - xSpeed);
            motorRotation(1, 1);
            analogWrite(PIN_MOTOR_PWM_LEFT, ySpeed);
            analogWrite(PIN_MOTOR_PWM_RIGHT, ySpeed - xSpeed);
            //digitalWrite(LMR, LOW); digitalWrite(RMR, LOW);
        }
        else
        {
            //Serial.println("Just Forward");
            //analogWrite(LMF, ySpeed); analogWrite(RMF, ySpeed);
            motorRotation(1, 1);
            analogWrite(PIN_MOTOR_PWM_LEFT, ySpeed);
            analogWrite(PIN_MOTOR_PWM_RIGHT, ySpeed);
            //digitalWrite(LMR, LOW); digitalWrite(RMR, LOW);
        }
    }

    else if (yValue <= (512 - deadzone))//Reverse
    {
        //Serial.println("Reverse");
        ySpeed = (512 - yValue) / 2;
        if(xValue > (512 + deadzone)) //Left
        {
            //Serial.println("Left");
            xSpeed = (xValue - 512) / 2;
            //digitalWrite(LMF, LOW); digitalWrite(RMF, LOW);
            //analogWrite(LMR, ySpeed - xSpeed); analogWrite(RMR, ySpeed);
            motorRotation(-1, -1);
            analogWrite(PIN_MOTOR_PWM_LEFT, ySpeed - xSpeed);
            analogWrite(PIN_MOTOR_PWM_RIGHT, ySpeed);
        }
        else if (xValue < (512 - deadzone)) //Right
        {
            //Serial.println("Right");
            xSpeed = (512 - xValue) / 2;
            //digitalWrite(LMF, LOW); digitalWrite(RMF, LOW);
            //analogWrite(LMR, ySpeed); analogWrite(RMR, ySpeed - xSpeed);
            motorRotation(-1, -1);
            analogWrite(PIN_MOTOR_PWM_LEFT, ySpeed);
            analogWrite(PIN_MOTOR_PWM_RIGHT, ySpeed - xSpeed);
        }
        else
        {
            //Serial.println("Just reverse");
            //digitalWrite(LMF, LOW); digitalWrite(RMF, LOW);
            //analogWrite(LMR, ySpeed); analogWrite(RMR, ySpeed);
            motorRotation(-1, -1);
            analogWrite(PIN_MOTOR_PWM_LEFT, ySpeed);
            analogWrite(PIN_MOTOR_PWM_RIGHT, ySpeed);
        }
    }

    else // X is between 512 +- deadzone
    {
        if(xValue > (512 + deadzone)) // zero point turn Left
        {
            //Serial.println("Zero point left");
            //digitalWrite(LMF, LOW); analogWrite(RMF, xSpeed);
            motorRotation(-1, 1);
            analogWrite(PIN_MOTOR_PWM_LEFT, xSpeed);
            analogWrite(PIN_MOTOR_PWM_RIGHT, xSpeed);
            //digitalWrite(LMR, LOW); digitalWrite(RMR, LOW);
        }
        else if(xValue < (512 - deadzone))// zero point turn Right
        {
            //Serial.println("Zero point right");
            //analogWrite(LMF, xSpeed); digitalWrite(RMF, LOW);
            motorRotation(1, -1);
            analogWrite(PIN_MOTOR_PWM_LEFT, xSpeed);
            analogWrite(PIN_MOTOR_PWM_RIGHT, xSpeed);
            //digitalWrite(LMR, LOW); digitalWrite(RMR, LOW);
        }
        else
        { // Full stop
            analogWrite(PIN_MOTOR_PWM_RIGHT,0);
            analogWrite(PIN_MOTOR_PWM_LEFT,0);
        }
    }
}

void handleLocalization() {
    cTime = millis();

    mpu.Execute();
    cTheta = mpu.GetAngZ()- thetaOffset;
    mySDR.updateLocation(lCounter, rCounter, cTheta);
    Serial.print("path;");
    Serial.print(mySDR.getXLocation());
    Serial.print(";");
    Serial.print(mySDR.getYLocation());
    Serial.print("\r\n");

    sSerial.print("path;");
    sSerial.print(mySDR.getXLocation());
    sSerial.print(";");
    sSerial.print(mySDR.getYLocation());
    sSerial.print("\r\n");
    //delay(100);
}

void handleSerialComm() {
    //  digitalWrite(LED_BUILTIN, HIGH);
//  if (Serial.available()){
//    Serial.print("[");
//    String msg = Serial.readStringUntil('\n\r');  // "Serial.readStringUntil('\n\r')" possiblly make a short delay if message didn't have '\r'"
//    Serial.print(msg);
//    Serial.println("]");
//  }
//  digitalWrite(LED_BUILTIN, LOW);

    if (rCounter != lastrCounter || lCounter != lastlCounter) {
        Serial.print("Encoder right ");
        Serial.print(rCounter);
        Serial.print("; Encoder left ");
        Serial.println(lCounter);

        lastlCounter = lCounter;
        lastrCounter = rCounter;
    }


    while (sSerial.available() > 0){
        char tmpChar = sSerial.read();
        if (msgBufPnt >= ATR_PROTOCOL_MAX_BUF_SIZE){
            Serial.println("Message Overflow");
            if ((tmpChar != '\n') || (tmpChar != '\r')){
                msgBuf[0] = tmpChar;
                msgBufPnt = 1;
            }
        }else{
            if ((tmpChar == '\n') || (tmpChar == '\r')){
                msgBuf[msgBufPnt] = '\0';
                checkMessage();
                msgBufPnt = 0;
            }else{
                msgBuf[msgBufPnt] = tmpChar;
                msgBufPnt++;
            }
        }
    }
}

bool checkMessage() {
//  Serial.println(msgBuf);
    char *p = msgBuf;
    String str;
    int cnt = 0;
// while ((str = strtok_r(p, ";", &p)) != NULL) // delimiter is the semicolon
    str = strtok_r(p, ";", &p);
    //Serial.println(str);
    if (str == "Joystick") {
        while ((str = strtok_r(p, ";", &p)) != nullptr) {
            if (cnt == 0) {          //joy x value
                joystick[0] = str.toInt();
            } else if (cnt == 1) {    //joy y value
                joystick[1] = str.toInt();
            } else if (cnt == 2) {    //joy sw value
                joystick[2] = str.toInt();
            }
            cnt++;
        }
//        Serial.println("Joy ");
//        Serial.print(joystick[0]);
//        Serial.print(" ");
//        Serial.print(joystick[1]);
//        Serial.print(" ");
//        Serial.println(joystick[2]);
    } else if (str == "Command") {

    } else if (str == "Message") {
        while ((str = strtok_r(p, ";", &p)) != nullptr) {
            Serial.print("[");
            Serial.print(str);
            Serial.println("]");
        }

    }
}

void checkLeftEncoder(){
    prevLeftCLK = nowLeftCLK;
    prevLeftDT = nowLeftDT;
    nowLeftCLK = digitalRead(PIN_WHEEL_ENCODER_CLK_LEFT);
    nowLeftDT = digitalRead(PIN_WHEEL_ENCODER_DT_LEFT);
    if ((prevLeftCLK == 0) && (prevLeftDT == 0)){
        if ((nowLeftCLK == 0) && (nowLeftDT == 1)){
            lCounter++;
        }else if ((nowLeftCLK == 1) && (nowLeftDT == 0)){
            lCounter--;
        }
    }else if ((prevLeftCLK == 0) && (prevLeftDT == 1)){
        if ((nowLeftCLK == 0) && (nowLeftDT == 0)){
            lCounter--;
        }else if ((nowLeftCLK == 1) && (nowLeftDT == 1)){
            lCounter++;
        }
    }else if ((prevLeftCLK == 1) && (prevLeftDT == 0)){
        if ((nowLeftCLK == 0) && (nowLeftDT == 0)){
            lCounter++;
        }else if ((nowLeftCLK == 1) && (nowLeftDT == 1)){
            lCounter--;
        }
    }else if ((prevLeftCLK == 1) && (prevLeftDT == 1)){
        if ((nowLeftCLK == 0) && (nowLeftDT == 1)){
            lCounter--;
        }else if ((nowLeftCLK == 1) && (nowLeftDT == 0)){
            lCounter++;
        }
    }
}



void checkRightEncoder(){
    prevRightCLK = nowRightCLK;
    prevRightDT = nowRightDT;
    nowRightCLK = digitalRead(PIN_WHEEL_ENCODER_CLK_RIGHT);
    nowRightDT = digitalRead(PIN_WHEEL_ENCODER_DT_RIGHT);
    if ((prevRightCLK == 0) && (prevRightDT == 0)){
        if ((nowRightCLK == 0) && (nowRightDT == 1)){
            rCounter++;
        }else if ((nowRightCLK == 1) && (nowRightDT == 0)){
            rCounter--;
        }
    }else if ((prevRightCLK == 0) && (prevRightDT == 1)){
        if ((nowRightCLK == 0) && (nowRightDT == 0)){
            rCounter--;
        }else if ((nowRightCLK == 1) && (nowRightDT == 1)){
            rCounter++;
        }
    }else if ((prevRightCLK == 1) && (prevRightDT == 0)){
        if ((nowRightCLK == 0) && (nowRightDT == 0)){
            rCounter++;
        }else if ((nowRightCLK == 1) && (nowRightDT == 1)){
            rCounter--;
        }
    }else if ((prevRightCLK == 1) && (prevRightDT == 1)){
        if ((nowRightCLK == 0) && (nowRightDT == 1)){
            rCounter--;
        }else if ((nowRightCLK == 1) && (nowRightDT == 0)){
            rCounter++;
        }
    }
}