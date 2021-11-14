// #include "Arduino.h"
#pragma once

#ifdef TEST
#include "arduino-mock/Arduino.h"
#endif

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"


typedef unsigned long time_t;

enum CarState
{
    OFF,
    RUNNING,
    MOVING
};

const unsigned long TURN_OFF_DELAY = 5000; // ms

const uint8_t START_PIN = 0x0;
const uint8_t FRONT_LIGHTS_PINS[] = {0x1, 0x2};

CarState car_state = CarState::OFF;

unsigned long last_move_timestamp = 0;



#pragma region MPU6050

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

MPU6050 mpu;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

#pragma endregion MPU6050


void setup()
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    Serial.begin(115200);
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(1150);
    mpu.setYAccelOffset(-50);
    mpu.setZAccelOffset(1060);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
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
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}


const byte msmrmntCnt = 10;
const byte timespan = 1.5; // s

byte yawAccelMeas[msmrmntCnt] = {}

void readaccel()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);


        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180 / M_PI);
    }
}

void loop()
{
    /*
    -- START CAR --
        1. The car turns on lights and plays engine rev sfx when held in hand
        2. The car then turns off lights after 5 sec if it hasn't moved or isn't being held anymore

    -- DRIVE CAR --
        1. The car plays dj gas sfx when being pushed forward
        2. The car plays break sfx when turned into a L after having moved forward

    */
    if (digitalRead(START_PIN) == HIGH)
    {
        last_move_timestamp = millis();

        if (car_state == CarState::OFF)
        {
            digitalWrite(FRONT_LIGHTS_PINS[0], HIGH);
            digitalWrite(FRONT_LIGHTS_PINS[1], HIGH);

            // TODO: play sfx

            car_state = CarState::RUNNING;
        }
    }
    else
    {
        if (millis() - last_move_timestamp >= TURN_OFF_DELAY)
        {
            car_state = CarState::OFF;
            digitalWrite(FRONT_LIGHTS_PINS[0], LOW);
            digitalWrite(FRONT_LIGHTS_PINS[1], LOW);
        }
    }

    // else if(car_state == CarState::RUNNING && last_move_time)
}
