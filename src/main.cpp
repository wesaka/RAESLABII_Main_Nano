#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyMPU6050.h>
#include <PID_v1.h>

/*
 * Updated for using the arduino nano as command
 */

/*
 * TODO for next milestone
 * Draw a square and make the robot go places inside the square
 * Also show the current location in that square via the OLED display
 * The OLED is 128 x 64 - use the entire screen as the "map"
 * Save the progress of the robot as a line in the map
 * Set up the nano (that will be the robot) bluetooth receiver as slave
 * And pair with the mega (that will be the controller)
 * See how that is made so they search for eachother
 *
 */

/*
 * Encoder variables
 */

#define ENCODER_DT_R 12
#define ENCODER_CLK_R 11
#define ENCODER_DT_L 10
#define ENCODER_CLK_L 9

/*
 * Global Variables
**/
int lastCounter[2] = {0, 0};

int currentStateCLK[2];
int lastStateCLK[2];

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
 * Encoder variables
 */

#define ENCODER_DT_R 12
#define ENCODER_CLK_R 11
#define ENCODER_DT_L 10
#define ENCODER_CLK_L 9

// Right is 0, Left is 1
int encoderCounter[2] = {0, 0}; // Just to keep in mind that one revolution is approx 30 "counts"

/*
 * DC Motor Variables
**/

#define M_ASPEED 3
#define M_AIN1 7
#define M_AIN2 8
#define M_STBY 6
#define M_BIN1 2
#define M_BIN2 4
#define M_BSPEED 5

bool isCW[2] = {true, true};

double odometer[2] = {0.0, 0.0};
double pwmSpeed[2] = {1.0, 1.0};
double speed[2] = {0.0, 0.0};
bool isIncreasing[2] = {true, true};

unsigned long currentRPM[2];

/*
 * PID
 */
double pidSetpoint, pidInput, pidOutput;
double Kp = 2, Ki = 5, Kd = 1;

PID pidR(&speed[0], &pwmSpeed[0], &pidSetpoint, Kp, Ki, Kd, DIRECT);
PID pidL(&speed[1], &pwmSpeed[1], &pidSetpoint, Kp, Ki, Kd, DIRECT);

/*
 * Control mode variables
 */
#define CONTROL_STANDBY 0
#define CONTROL_AUTO 1
#define CONTROL_REMOTE 2

int controlMode = CONTROL_STANDBY;

/*
 * OLED Variables
 **/
bool ALLOCATED = false;
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*
 * Joystick Variables
 **/
//int JOY_X = A8;
//int JOY_Y = A9;

/*
 * MPU
 */

MPU6050 mpu(Wire);

/*
 * Buttons variables
 */
#define B_1 53
#define B_2 65
#define B_3 64
#define B_UP 61
#define B_DOWN 60
#define B_LEFT 59
#define B_RIGHT 58

/*
 * Task variables
 */
int currentTask = 0;
String currentCommand = "";

#define TARGET_SPEED 1
#define TARGET_ODOMETER 2


/*
 * Function declarations
 */

void getRPM();

double getSpeed(int motor);

double getOdometer(int motor);

void getEncoderCount(int motor);

void handleOLED();

void handleSerialComm();

void handleAsync();

void autoMotor(int motor);

void stopAll();

void targetOdometer(double, int);

void targetSpeed(double);

/*
 * General functions
 */
int selectLine(int line) { return line * 8; }


/*
 * Main Arduino functions
 */

void setup() {
    Serial.begin(9600);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
    }

    Serial.println("OLED allocated successfully.");
    ALLOCATED = true;


// Set encoder pins as inputs
    pinMode(ENCODER_CLK_R, INPUT);
    pinMode(ENCODER_DT_R, INPUT);

    pinMode(M_STBY, OUTPUT);
    digitalWrite(M_STBY, HIGH);

    // Set up motor pins - A
    pinMode(M_ASPEED, OUTPUT);
    pinMode(M_AIN1, OUTPUT);
    pinMode(M_AIN2, OUTPUT);

    // Set up motor pins - B
    pinMode(M_BSPEED, OUTPUT);
    pinMode(M_BIN1, OUTPUT);
    pinMode(M_BIN2, OUTPUT);

    // Read the initial state of CLK
    lastStateCLK[0] = digitalRead(ENCODER_CLK_R);
    lastStateCLK[1] = digitalRead(ENCODER_CLK_L);

    // Joystick switch needs to have pullup
//    pinMode(B_1, INPUT_PULLUP);
//    pinMode(B_2, INPUT_PULLUP);
//    pinMode(B_3, INPUT_PULLUP);
//    pinMode(B_UP, INPUT_PULLUP);
//    pinMode(B_DOWN, INPUT_PULLUP);
//    pinMode(B_RIGHT, INPUT_PULLUP);
//    pinMode(B_LEFT, INPUT_PULLUP);
//    pinMode(JOY_Y, INPUT);
//    pinMode(JOY_X, INPUT);

    // IMU init
    mpu.Initialize();

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
    // Doing 10 micros for encoder
    if (micros() > tenMicros + 10) {
        tenMicros = micros();
        getEncoderCount(0);
        getEncoderCount(1);
    }

    if (millis() > oneMillis + 1) {
        oneMillis = millis();

        getRPM();
    }


    if (millis() > hundredMillis + 100) {
        // Reset the global time variable to reflect now
        hundredMillis = millis();
        //Serial.println(hundredMillis);

        // Handlers
        handleSerialComm();
        handleOLED();

        // todo make an auto mode for testing for now
    }
}

void getRPM() {
    /*
     * To calculate the RPM I'll use 1 revolution of the wheel, that is approx 30 in the "encoderCounter"
     * Since every 30 the wheel does one revolution, but that's not exact :/
     */

    //Serial.println(encoderCounter);

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
    int motorAddress = (motor == 0) ? ENCODER_CLK_R : ENCODER_CLK_L;
    currentStateCLK[motor] = digitalRead(motorAddress);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK[motor] != lastStateCLK[motor] && currentStateCLK[motor] == 1) {

        // If the DT state is different than the CLK state then
        // the encoder is rotating CCW so decrement
        if (digitalRead(ENCODER_DT_R) != currentStateCLK[motor]) {
            encoderCounter[motor]--;

        } else {
            // Encoder is rotating CW so increment
            encoderCounter[motor]++;

        }
    }

    // Remember last CLK state
    lastStateCLK[motor] = currentStateCLK[motor];
}

void handleOLED() {
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
}

String receiveSerialString() {
    const int inputSize = 50;
    char inputArray[inputSize];
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    // if (Serial.available() > 0) {
    while (Serial.available() > 0) {
        rc = Serial.read();

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

/**
 * This is useful for both outputting messages in the serial monitor
 * And for getting input from there
 * We can use it for controlling the menus
 * For now, just the speedometer, odometer and orientation information are available
 * And since we are integrating it with the bluetooth controller
 * This is the central function for controlling the behavior of the robot
 * This will determine what it will do, basically
 *
 * Input in the serial monitor:
 * a: for automatic - increases and decreases speed automatically
 * m: for manual - still have to implement it
 *
 * s: for setting the target speed
 * o: for setting the target odometer reading
 * r: to see orientation stats in OLED
 * i: to get that information in serial monitor
 */
void handleSerialComm() {
    /*
     * This used for inputting data from serial
     * Use with Serial.readString()
     * inputString.startsWith("whatever")
     */

    // TODO figure out what is messing with the OLED allocation

    if (Serial.available() > 0) {
        String option = receiveSerialString();

        Serial.print("Received string: ");
        Serial.println(option);

        currentCommand = option;

//        // Process each level of the command sent
//        char* level;
//        level = strtok(option, "/");
//        while (level != NULL) {
//            // Do something with the level
//            // TODO add more complexity to the proccessing of commands
//        }

        if(option.startsWith("odom")) {
            currentTask = TARGET_ODOMETER;
        } else if (option.startsWith("speed")) {
            currentTask = TARGET_SPEED;
        } else if (option.startsWith("P")) {
            Kp = option.substring(2).toDouble();
        } else if (option.startsWith("I")) {
            Ki = option.substring(2).toDouble();
        } else if (option.startsWith("D")) {
            Kd = option.substring(2).toDouble();
        } else if (option.startsWith("info")) {
            char toWrite[100];
            snprintf(toWrite, 100, "Current RPM: %lu R - %lu L\n"
                                  "Current PWM: %f R - %f L\n"
                                  "Current speed (meters/minute): %f R - %f L\n"
                                  "Current odometer reading : %f R - %f L\n",
                                  currentRPM[0], currentRPM[1],
                                  pwmSpeed[0], pwmSpeed[1],
                                  getSpeed(0), getSpeed(1),
                                  getOdometer(0), getOdometer(1));

            Serial.println(toWrite);

            //IMU checking
            Serial.print("[");
            mpu.Execute();
            Serial.print(mpu.GetAngX());
            Serial.print("  ");
            Serial.print(mpu.GetAngY());
            Serial.print("  ");
            Serial.print(mpu.GetAngZ());
            Serial.println("]");
        } else {
            currentCommand = "";
            currentTask = 0;
        }
    }
}

/*
 * Actuators
 */

void autoMotor(int motor) {
    // Handle the increase in speed automatically
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

    digitalWrite(M_STBY, HIGH);

    int in1Address = (motor == 0) ? M_AIN1 : M_BIN1;
    int in2Address = (motor == 0) ? M_AIN2 : M_BIN2;
    int speedAddress = (motor == 0) ? M_ASPEED : M_BSPEED;

    if (isCW[motor]) {
        digitalWrite(in1Address, HIGH);
        digitalWrite(in2Address, LOW);
    } else {
        digitalWrite(in1Address, LOW);
        digitalWrite(in2Address, HIGH);
    }

    analogWrite(speedAddress, pwmSpeed[motor]);

//    Serial.print("Current speed: ");
//    Serial.println(speed);
}

void stopAll() {
    digitalWrite(M_STBY, LOW);
}

/**
 * This functions controls the wheel to run at full speed until reaches the desired odometer
 *
 * @param odometerReading The target total distance traveled
 */
void targetOdometer(double odometerReading, int side) {
    if (odometerReading <= odometer[side]) {
        digitalWrite(M_STBY, LOW);
    } else {
        digitalWrite(M_STBY, HIGH);
        digitalWrite((side == 0) ? M_AIN1 : M_BIN1, HIGH);
        digitalWrite((side == 0) ? M_AIN2 : M_BIN2, LOW);
        analogWrite((side == 0) ? M_ASPEED : M_BSPEED, 255);
    }
}

/**
 * The function that when called sets the robot to run at a certain speed
 *
 * @param speedReading Accepts the speed value to which you want the robot to maintain
 */
void targetSpeed(double speedReading) {
    digitalWrite(M_STBY, HIGH);
    digitalWrite(M_AIN1, HIGH);
    digitalWrite(M_AIN2, LOW);
    digitalWrite(M_BIN1, HIGH);
    digitalWrite(M_BIN2, LOW);

    for (int i = 0; i < 2; i++) {
        // Ramp up the speed until we hit the desired value
        // Use PID for that
        pidSetpoint = speedReading;

        speed[i] = getSpeed(i);
        (i == 0 ? pidR : pidL).Compute();
        analogWrite((i == 0) ? M_ASPEED : M_BSPEED, pwmSpeed[i]);
    }

}