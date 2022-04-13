#include <Arduino.h>
#include <Wire.h>
#include <TinyMPU6050.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

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
 * Global Variables
**/
int lastCounter[2] = {0, 0};
int currentPos[2] = {0, 0};
int targetPos[2] = {0, 0};

int currentStateCLK[2];
int lastStateCLK[2];

/*
 * Menu Strings
 */

#define MENU_MODE "Mode"
#define MENU_AUTORUN "Autorun"
#define MENU_REMOTE_CONTROL "Remote-Control"
#define MENU_SPEEDOMETER "Speedometer"
#define MENU_ODOMETER "Odometer"
#define MENU_ORIENTATION "Orientation"
#define MENU_CURRENT_LOCATION "Current-Location"
#define MENU_REMOTE_CONTROL_MESSAGE "Remote-Control-Message"
#define MENU_SYSTEM_INFO "System-Info"

/*
 * Timekeeping
**/
unsigned long timeForRPM = millis();

/*
 * TODO menu values are going to be done using strings - those ints were too convoluted
 * Possible values:
 * [Mode]  System-Info | Remote-Control | Autorun
 * [Mode/Autorun] Speedmeter (X m/h) | Odometer |  Orentation | Current Location x,y |
 * [Mode/Remote-Control] Speedmeter (X m/h) | Odometer | Orentation | Current Location x,y | Remote-Control-Message
 * [Mode/System-Info ] Wheel-Diameter | Distance Between two wheels
 */

#define SPACE "   "

/*
 * Encoder/Motor variables
 */

#define PIN_WHEEL_ENCODER_CLK_RIGHT    3
#define PIN_WHEEL_ENCODER_DT_RIGHT    5
#define PIN_MOTOR_PWM_RIGHT    11
#define PIN_MOTOR_IN0_RIGHT    10
#define PIN_MOTOR_IN1_RIGHT    9

#define PIN_WHEEL_ENCODER_CLK_LEFT    2
#define PIN_WHEEL_ENCODER_DT_LEFT    4
#define PIN_MOTOR_PWM_LEFT    6
#define PIN_MOTOR_IN0_LEFT    7
#define PIN_MOTOR_IN1_LEFT    8

// Right is 0, Left is 1
int encoderCounter[2] = {0, 0}; // Just to keep in mind that one revolution is approx 30 "counts"
int encoderTarget[2] = {0, 0};

bool isCW[2] = {true, true};

double odometer[2] = {0.0, 0.0};
double pwmSpeed[2] = {1.0, 1.0};
double realSpeed[2] = {0.0, 0.0};
bool isIncreasing[2] = {true, true};

unsigned long currentRPM[2];

/*
 * PID
 */
double pidSetpoint, pidInput, pidOutput;
double Kp = 2, Ki = 5, Kd = 1;

PID pidR(&realSpeed[0], &pwmSpeed[0], &pidSetpoint, Kp, Ki, Kd, DIRECT);
PID pidL(&realSpeed[1], &pwmSpeed[1], &pidSetpoint, Kp, Ki, Kd, DIRECT);

/*
 * Control mode variables
 */
#define CONTROL_STANDBY 0
#define CONTROL_AUTO 1
#define CONTROL_REMOTE 2

// Start off in remote control
int controlMode = CONTROL_REMOTE;

/*
 * MPU
 */

MPU6050 mpu(Wire);

/*
 * Task variables
 */
String currentCommandString = "";

#define TARGET_SPEED 1
#define TARGET_ODOMETER 2


/*
 * Function declarations
 */

void getRPM();

double getSpeed(int);

double getOdometer(int);

void getEncoderCount(int);

void handleSerialComm();

void handleAsync();

void handleBehavior();

void autoMotor(int);

void stopAll();

void targetOdometer(double, int);

void targetSpeed(double);

void turn(float degrees);

void doADonut();

void handleEncoderTarget(int speed);

void driveTo(int x, int y);

/*
 * Main Arduino functions
 */

SoftwareSerial sSerial = SoftwareSerial(13, 12);
void setup() {
    sSerial.begin(38400);

// Set encoder pins as inputs
    pinMode(PIN_WHEEL_ENCODER_CLK_RIGHT, INPUT_PULLUP);
    pinMode(PIN_WHEEL_ENCODER_DT_RIGHT, INPUT);
    pinMode(PIN_WHEEL_ENCODER_CLK_LEFT, INPUT_PULLUP);
    pinMode(PIN_WHEEL_ENCODER_DT_LEFT, INPUT);

    // Set up motor pins - A
    pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
    pinMode(PIN_MOTOR_IN0_RIGHT, OUTPUT);
    pinMode(PIN_MOTOR_IN1_RIGHT, OUTPUT);

    // Set up motor pins - B
    pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
    pinMode(PIN_MOTOR_IN0_LEFT, OUTPUT);
    pinMode(PIN_MOTOR_IN1_LEFT, OUTPUT);

    // Read the initial state of CLK
    lastStateCLK[0] = digitalRead(PIN_WHEEL_ENCODER_CLK_RIGHT);
    lastStateCLK[1] = digitalRead(PIN_WHEEL_ENCODER_CLK_LEFT);

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
}

void loop() {
    handleAsync();
}

unsigned long hundredMillis = millis();
unsigned long oneMillis = millis();
unsigned long tenMicros = micros();

void handleAsync() {
    if (millis() > oneMillis + 10) {
        oneMillis = millis();

        getEncoderCount(0);
        getEncoderCount(1);
        getRPM();
    }


    if (millis() > hundredMillis + 100) {
        // Reset the global time variable to reflect now
        hundredMillis = millis();
        //sSerial.println(hundredMillis);

        // Handlers
        handleSerialComm();
        handleBehavior();
        handleEncoderTarget(100);

        // todo make an auto mode for testing for now
    }
}

/**
 * This is the function that determines what the robot will do
 */

void handleBehavior() {
    switch (controlMode) {
        case CONTROL_AUTO:
            break;

        case CONTROL_REMOTE:
            break;

        default:
            break;
    }
}

void getRPM() {
    /*
     * To calculate the RPM I'll use 1 revolution of the wheel, that is approx 30 in the "encoderCounter"
     * Since every 30 the wheel does one revolution, but that's not exact :/
     */

    //sSerial.println(encoderCounter);

    for (int i = 0; i < 2; i++) {
        int counterResult = 0;

        if (encoderCounter[i] > lastCounter[i]) {
            counterResult = encoderCounter[i] - lastCounter[i];
        } else if (encoderCounter[i] < lastCounter[i]) {
            counterResult = lastCounter[i] - encoderCounter[i];
        }

        if (counterResult == 0) {
            currentRPM[i] = 0;

        } else if (counterResult >= 30) {
            // 1 revolution have passed
            lastCounter[i] = encoderCounter[i];

            // Add it to the odometer
            odometer[i] += 0.215;

            // Get the time it took
            unsigned long elapsedTime = millis() - timeForRPM;

            timeForRPM = millis();

            currentRPM[i] = (1 * (60000 / elapsedTime));
        }
    }

}

double getSpeed(int motor) {
    // Returns in meters/hour
    return (0.215 * (float) currentRPM[motor]) * 60;
}

double getOdometer(int motor) {
    return odometer[motor]; // Returns in total meters traveled
}

void getEncoderCount(int motor) {
    // Read the current state of CLK
    currentStateCLK[motor] = digitalRead((motor == 0) ? PIN_WHEEL_ENCODER_CLK_RIGHT : PIN_WHEEL_ENCODER_CLK_LEFT);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK[motor] != lastStateCLK[motor] && currentStateCLK[motor] == 1) {

        // If the DT state is different than the CLK state then
        // the encoder is rotating CCW so decrement
        if (digitalRead((motor == 0) ? PIN_WHEEL_ENCODER_DT_RIGHT : PIN_WHEEL_ENCODER_DT_LEFT) != currentStateCLK[motor]) {
            encoderCounter[motor] += (-1 + (2 * motor)) ;

        } else {
            // Encoder is rotating CW so increment
            encoderCounter[motor] += (1 - (2 * motor));

        }
    }

    // Remember last CLK state
    lastStateCLK[motor] = currentStateCLK[motor];
}

// Todo improve this, as this is going to be the main way to control the car, except when it is on auto mode
String receiveSerialString() {
    const int inputSize = 50;
    char inputArray[inputSize];
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    while (sSerial.available() > 0) {
        rc = sSerial.read();

        if (rc != endMarker) {
            inputArray[ndx] = rc;
            ndx++;
            if (ndx >= inputSize) {
                ndx = inputSize - 1;
            }
        }
        else {
            inputArray[ndx] = '\0'; // terminate the string
            ndx = 0;
        }
    }

    return {inputArray};
}

void handleSerialComm() {
    /*
     * This used for inputting data from serial
     * Use with sSerial.readString()
     * inputString.startsWith("whatever")
     */

    // TODO figure out what is messing with the OLED allocation

    if (sSerial.available()) {
        String option = sSerial.readString();
//        Serial.println(option);

        // Sanitize the string received if needed and set currentCommandString
        // currentCommandString is the variable that holds the full "path" of what we should be doing
        // First, check on what level we are, counting the number of '/' we have - not sure if I will really need this
        int level = 0;
        for (int i = 0; option[i]; i++)
            if (option[i]=='/') level++;

        currentCommandString = option;

        // TODO finish this

        if (currentCommandString.indexOf(MENU_MODE) != -1) {
            // Manual mode check

        } else if (currentCommandString.indexOf(MENU_REMOTE_CONTROL) != -1) {
            // Manual mode check - remote controlled
            controlMode = CONTROL_REMOTE;
            sSerial.print("Control Remote Set");

        } else if (currentCommandString.indexOf(MENU_AUTORUN) != -1) {
            // Autorun mode check
            controlMode = CONTROL_AUTO;
            sSerial.print("Control Auto Set");

        } else if (currentCommandString.indexOf(MENU_SPEEDOMETER) != -1) {
            // If we want a speedometer
            // Check if autorun or manual first

        } else if (currentCommandString.indexOf(MENU_ODOMETER) != -1) {
            // If we want the odometer
            // Check if autorun or manual first
            sSerial.print("Odometer right - ");
            sSerial.println(getOdometer(0));
            sSerial.print("Odometer left - ");
            sSerial.println(getOdometer(1));

        } else if (currentCommandString.indexOf(MENU_ORIENTATION) != -1) {
            // If we want the orientation
            // Check if autorun or manual first
            mpu.Execute();
            sSerial.print("AngX = ");
            sSerial.print(mpu.GetAngX());
            sSerial.print("  /  AngY = ");
            sSerial.print(mpu.GetAngY());
            sSerial.print("  /  AngZ = ");
            sSerial.println(mpu.GetAngZ());

        } else if (currentCommandString.indexOf(MENU_CURRENT_LOCATION) != -1) {
            // If we want the current location
            // Check if autorun or manual first

        } else if (currentCommandString.indexOf(MENU_REMOTE_CONTROL_MESSAGE) != -1) {
            // Get a message from serial

        } else if (currentCommandString.indexOf(MENU_SYSTEM_INFO) != -1) {
            char toWrite[100];
            snprintf(toWrite, 100, "Current RPM: %lu R - %lu L\n"
                                   "Current PWM: %f R - %f L\n"
                                   "Current realSpeed (meters/minute): %f R - %f L\n"
                                   "Current odometer reading : %f R - %f L\n",
                     currentRPM[0], currentRPM[1],
                     pwmSpeed[0], pwmSpeed[1],
                     getSpeed(0), getSpeed(1),
                     getOdometer(0), getOdometer(1));

            sSerial.println(toWrite);

            //IMU checking
            sSerial.print("[");
            mpu.Execute();
            sSerial.print(mpu.GetAngX());
            sSerial.print("  ");
            sSerial.print(mpu.GetAngY());
            sSerial.print("  ");
            sSerial.print(mpu.GetAngZ());
            sSerial.println("]");


            // Also have to check if PID is asked to change
            if (currentCommandString.indexOf("P") != -1) {
                Kp = option.substring(2).toDouble();
            } else if (currentCommandString.indexOf("I") != -1) {
                Ki = option.substring(2).toDouble();
            } else if (currentCommandString.indexOf("D") != -1) {
                Kd = option.substring(2).toDouble();
            }
        } else {
            // It should reach here if it is not a "directory" change
            if (currentCommandString.indexOf("Donut") != -1) {
                doADonut();
            } else if (currentCommandString.indexOf("Target") != -1 && controlMode == CONTROL_REMOTE) {
                // Get the X and Y coordinates of the target
                // The user should input the command in the bluetooth serial as
                // TargetX123|Y123 - where 123 are the desired coordinates
                int xValue = currentCommandString.substring(currentCommandString.indexOf("X") + 1, currentCommandString.indexOf("|")).toInt();
                int yValue = currentCommandString.substring(currentCommandString.indexOf("Y") + 1, currentCommandString.length()).toInt();
                sSerial.print("Routing to X Y");
                sSerial.print(xValue);
                sSerial.print(yValue);

            } else if (currentCommandString.indexOf("Turn") != -1 && controlMode == CONTROL_REMOTE) {
                // Turn a set amount of degrees inplace
                // Expecting Turn:123 where 123 are the degrees
                sSerial.print("Turning degrees: ");
                sSerial.print(currentCommandString.substring(currentCommandString.indexOf(":") + 1, currentCommandString.length()));

                float degrees = currentCommandString.substring(currentCommandString.indexOf(":") + 1, currentCommandString.length()).toFloat();
                turn(degrees);

            } else if (currentCommandString.indexOf("Stop") != -1 && controlMode == CONTROL_REMOTE) {
                // Set the target to be the actual value
                encoderTarget[0] = encoderCounter[0];
                encoderTarget[1] = encoderCounter[1];
            }
        }
    }
}

/*
 * Actuators
 */

void autoMotor(int motor) {
    // Handle the increase in realSpeed automatically
    int speedStep = isIncreasing[motor] ? 1 : -1;
    pwmSpeed[motor] += speedStep;

    if (pwmSpeed[motor] > 255) {
        pwmSpeed[motor] = 255;
        isIncreasing[motor] = false;
        isCW[motor] = false;
    }

    if (pwmSpeed[motor] < 50) {
        pwmSpeed[motor] = 50;
        isIncreasing[motor] = true;
        isCW[motor] = true;
    }

    int in1Address = (motor == 0) ? PIN_MOTOR_IN0_RIGHT : PIN_MOTOR_IN0_LEFT;
    int in2Address = (motor == 0) ? PIN_MOTOR_IN1_RIGHT : PIN_MOTOR_IN1_LEFT;
    int speedAddress = (motor == 0) ? PIN_MOTOR_PWM_RIGHT : PIN_MOTOR_PWM_LEFT;

    if (isCW[motor]) {
        digitalWrite(in1Address, HIGH);
        digitalWrite(in2Address, LOW);
    } else {
        digitalWrite(in1Address, LOW);
        digitalWrite(in2Address, HIGH);
    }

    analogWrite(speedAddress, pwmSpeed[motor]);

//    sSerial.print("Current realSpeed: ");
//    sSerial.println(realSpeed);
}

void stopAll() {
    analogWrite(PIN_MOTOR_PWM_RIGHT , 0);
    analogWrite(PIN_MOTOR_PWM_LEFT , 0);
}

/**
 * This functions controls the wheel to run at full realSpeed until reaches the desired odometer
 *
 * @param odometerReading The target total distance traveled
 */
void targetOdometer(double odometerReading, int side) {
    if (odometerReading <= odometer[side]) {
        analogWrite((side == 0) ? PIN_MOTOR_PWM_RIGHT : PIN_MOTOR_PWM_LEFT, 0);
    } else {
        digitalWrite((side == 0) ? PIN_MOTOR_IN0_RIGHT : PIN_MOTOR_IN0_LEFT, HIGH);
        digitalWrite((side == 0) ? PIN_MOTOR_IN1_RIGHT : PIN_MOTOR_IN1_LEFT, LOW);
        analogWrite((side == 0) ? PIN_MOTOR_PWM_RIGHT : PIN_MOTOR_PWM_LEFT, 255);
    }
}

/**
 * The function that when called sets the robot to run at a certain realSpeed
 *
 * @param speedReading Accepts the realSpeed value to which you want the robot to maintain
 */
void targetSpeed(double speedReading) {
    //digitalWrite(M_STBY, HIGH); // STDBY is bound to 5v
    digitalWrite(PIN_MOTOR_IN0_RIGHT, HIGH);
    digitalWrite(PIN_MOTOR_IN1_RIGHT, LOW);
    digitalWrite(PIN_MOTOR_IN0_LEFT, HIGH);
    digitalWrite(PIN_MOTOR_IN1_LEFT, LOW);

    for (int i = 0; i < 2; i++) {
        // Ramp up the realSpeed until we hit the desired value
        // Use PID for that
        pidSetpoint = speedReading;

        realSpeed[i] = getSpeed(i);
        (i == 0 ? pidR : pidL).Compute();
        analogWrite((i == 0) ? PIN_MOTOR_PWM_RIGHT : PIN_MOTOR_PWM_LEFT, pwmSpeed[i]);
    }

}

void doADonut() {

}

/**
 * Use this function to manually set how many counts you want the motor in question to work to
 *
 * @param motor 0 Right - 1 Left
 * @param count How many more counts to drive to
 * @param speed How fast?
 */
void handleEncoderTarget(int speed) {
    for (int i = 0; i < 2; i++) {
        int pinSpeed = (i == 0) ? PIN_MOTOR_PWM_RIGHT : PIN_MOTOR_PWM_LEFT;
        int toGo = encoderTarget[i] > encoderCounter[i] ?  encoderTarget[i] - encoderCounter[i] : encoderCounter[i] - encoderTarget[i];

        //TODO figure out a way to reliably stop the motor when we get where we want
        if (toGo <= 2) {
            encoderTarget[i] = encoderCounter[i];
            analogWrite(pinSpeed, 0);
        } else {
            analogWrite(pinSpeed, speed);

            int pinIn1 = (i == 0) ? PIN_MOTOR_IN0_RIGHT : PIN_MOTOR_IN0_LEFT;
            int pinIn2 = (i == 0) ? PIN_MOTOR_IN1_RIGHT : PIN_MOTOR_IN1_LEFT;

            int count = encoderTarget[i] - encoderCounter[i];
            if (count > 0) {
                // Forwards
                digitalWrite(pinIn1, HIGH);
                digitalWrite(pinIn2, LOW);

            } else if (count < 0) {
                // Backwards
                digitalWrite(pinIn1, LOW);
                digitalWrite(pinIn2, HIGH);

            }
        }
    }
}

/**
 * This is the turning function. It turns the thing
 * Here's the deal, I do it using the encoder count, and keeping in mind that for approx 30 counts
 * we have a full wheel turn, and when the wheel does one full turn it travels 225mm
 * For half a turn (180 degrees), both wheels have to travel 262mm - so this is approx equal to 35 counts
 * This is the maximum it turns, so we have to calculate the values in the middle
 * Each count is approx equal to 5 degrees
 *
 * @param degrees How many degrees you want it to turn inplace - trig circle, it works like that
 */
void turn(float degrees) {
    sSerial.print("This is indeed the last version...\n");

    // First, get how many degrees and choose which way to turn
    int direction = 1;
    float degreesToTurn = degrees;
    if (degrees > 180) {
        direction = -1;
        degreesToTurn = degrees - 180;
    }

    sSerial.print("Turning ");
    sSerial.println(degreesToTurn);

    int numberOfCounts = floor(degreesToTurn / 5.14) * direction;
    encoderTarget[0] = (encoderCounter[0] + numberOfCounts);
    encoderTarget[1] = (encoderCounter[1] + (numberOfCounts * -1));
}

/**
 * This is the main function that drives the robot
 * I have defined that 1 unit in any direction is a full revolution of the wheel.
 * For now it will drive in a straight line, curves and stuff might be split into smaller sections
 * Imagine that the robot is at the center of a XY plan
 * And you're looking at it from a top down view
 * Positive X is wherever the robot is looking at
 * IMU is not reliable enough for setting this :(
 *
 * @param x X position to drive to
 * @param y Y position to drive to
 */
void driveTo(int x, int y) {

}