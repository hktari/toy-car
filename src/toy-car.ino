#ifdef TEST
#pragma once
#include "arduino-mock/Arduino.h"
#endif

#include <AceRoutine.h>
using namespace ace_routine;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <math.h>
#include <Fastwire.h>

typedef unsigned long time_t;

class Button
{
    int prev_state;
    int cur_state;

public:
    const int pin;

public:
    Button(int pin) : pin(pin)
    {
        prev_state = cur_state = 0;
    }

    void update()
    {
        set_state(digitalRead(pin));
    }

    bool transitioned_to(int state)
    {
        return prev_state != state && cur_state == state;
    }

    int get_state()
    {
        return cur_state;
    }

private:
    void set_state(int state)
    {
        prev_state = cur_state;
        cur_state = state;
    }
};

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

#define LED_FORWARD_PIN 3
#define LED_BACKWARDS_PIN 5
#define THRESHOLD_STEP_BTN_PIN 4

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
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

MPU6050 mpu;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

#define I2CDEV_IMPLEMENTATION I2CDEV_BUILTIN_FASTWIRE

void setup_mpu6050()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Serial.println("FASTWIRE");
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    // while (!Serial)
    //     ; // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
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
}
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_FORWARD_PIN, OUTPUT);
    pinMode(LED_BACKWARDS_PIN, OUTPUT);
    pinMode(THRESHOLD_STEP_BTN_PIN, INPUT_PULLUP);

    digitalWrite(LED_FORWARD_PIN, HIGH);
    digitalWrite(LED_BACKWARDS_PIN, HIGH);

    setup_mpu6050();

    delay(500);
}

void readaccel()
{
}

VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorInt16 prev_world_accel;

int16_t accel_threshold = 50;
Button thresholdStepBtn(THRESHOLD_STEP_BTN_PIN);

void accel_threshold_step()
{
    accel_threshold += 50;
    Serial.print("Accel threshold: ");
    Serial.print(accel_threshold, DEC);
    Serial.println();
}

COROUTINE(accelLEDHandler)
{
    COROUTINE_LOOP()
    {
        // int16_t accel_delta = aaWorld.x - prev_world_accel.x;
        // Serial.print("delta: ");
        // Serial.print(accel_delta);
        // Serial.println();

        int16_t acceldelta = aaWorld.x;

        if (acceldelta < 0)
        {
            acceldelta = aaWorld.x * -1;
        }

        if (acceldelta > 255)
        {
            acceldelta = 255;
        }

        analogWrite(LED_FORWARD_PIN, acceldelta);
        // else
        // {
        //     digitalWrite(LED_FORWARD_PIN, LOW);
        // }

        // if (accel_delta < 0)
        // {
        //     digitalWrite(LED_BACKWARDS_PIN, HIGH);
        // }

        delay(100);

        // prev_world_accel = aaWorld; // copy
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
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // if (mpuInterrupt)
    // {
    //     // Serial.println("Data received.");
    // }

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
        // Serial.print("\t");
        // Serial.print(aaWorld.y);
        // Serial.print("\t");
        // Serial.println(aaWorld.z);
        Serial.println();
        // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Serial.print("ypr\t");
        // Serial.print(ypr[0] * 180 / M_PI);
        // Serial.print("\t");
        // Serial.print(ypr[1] * 180 / M_PI);
        // Serial.print("\t");
        // Serial.print(ypr[2] * 180 / M_PI);
        // digitalWrite(LED_PIN, led
    }

    // accelLEDHandler.runCoroutine();

    int16_t acceldelta = aaWorld.x;

    if (acceldelta < 0)
    {
        acceldelta = aaWorld.x * -1;
    }

    if (acceldelta > 255)
    {
        acceldelta = 255;
    }

    analogWrite(LED_FORWARD_PIN, acceldelta);

    // thresholdStepBtn.update();
    // if (thresholdStepBtn.transitioned_to(LOW))
    // {
    //     accel_threshold_step();
    // }

    // mpuInterrupt = false;

    delay(100);

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
