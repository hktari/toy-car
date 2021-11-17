#ifdef TEST
#pragma once
#include "arduino-mock/Arduino.h"
#endif

#include <AceRoutine.h>
using namespace ace_routine;

#include <math.h>
#include <Wire.h>
#include <MPU6050.h>

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
#define LED_ALIVE_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool led_alive_state = false;

void setup()
{
    pinMode(LED_ALIVE_PIN, OUTPUT);
    pinMode(LED_FORWARD_PIN, OUTPUT);
    pinMode(LED_BACKWARDS_PIN, OUTPUT);
    pinMode(THRESHOLD_STEP_BTN_PIN, INPUT_PULLUP);

    digitalWrite(LED_FORWARD_PIN, HIGH);
    digitalWrite(LED_BACKWARDS_PIN, HIGH);

    setup_mpu6050();

    delay(500);
}

MPU6050 mpu;

void setup_mpu6050()
{
    while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

    mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);

    mpu.setIntFreeFallEnabled(false);
    mpu.setIntZeroMotionEnabled(false);
    mpu.setIntMotionEnabled(false);

    mpu.setDHPFMode(MPU6050_DHPF_5HZ);

    mpu.setMotionDetectionThreshold(4);
    mpu.setMotionDetectionDuration(5);

    mpu.setZeroMotionDetectionThreshold(4);
    mpu.setZeroMotionDetectionDuration(2);

    Serial.println();

    Serial.print(" * Sleep Mode:                ");
    Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Motion Interrupt:     ");
    Serial.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Zero Motion Interrupt:     ");
    Serial.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Free Fall Interrupt:       ");
    Serial.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Motion Threshold:          ");
    Serial.println(mpu.getMotionDetectionThreshold());

    Serial.print(" * Motion Duration:           ");
    Serial.println(mpu.getMotionDetectionDuration());

    Serial.print(" * Zero Motion Threshold:     ");
    Serial.println(mpu.getZeroMotionDetectionThreshold());

    Serial.print(" * Zero Motion Duration:      ");
    Serial.println(mpu.getZeroMotionDetectionDuration());

    Serial.print(" * Clock Source:              ");
    switch (mpu.getClockSource())
    {
    case MPU6050_CLOCK_KEEP_RESET:
        Serial.println("Stops the clock and keeps the timing generator in reset");
        break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ:
        Serial.println("PLL with external 19.2MHz reference");
        break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ:
        Serial.println("PLL with external 32.768kHz reference");
        break;
    case MPU6050_CLOCK_PLL_ZGYRO:
        Serial.println("PLL with Z axis gyroscope reference");
        break;
    case MPU6050_CLOCK_PLL_YGYRO:
        Serial.println("PLL with Y axis gyroscope reference");
        break;
    case MPU6050_CLOCK_PLL_XGYRO:
        Serial.println("PLL with X axis gyroscope reference");
        break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:
        Serial.println("Internal 8MHz oscillator");
        break;
    }

    Serial.print(" * Accelerometer:             ");
    switch (mpu.getRange())
    {
    case MPU6050_RANGE_16G:
        Serial.println("+/- 16 g");
        break;
    case MPU6050_RANGE_8G:
        Serial.println("+/- 8 g");
        break;
    case MPU6050_RANGE_4G:
        Serial.println("+/- 4 g");
        break;
    case MPU6050_RANGE_2G:
        Serial.println("+/- 2 g");
        break;
    }

    Serial.print(" * Accelerometer offsets:     ");
    Serial.print(mpu.getAccelOffsetX());
    Serial.print(" / ");
    Serial.print(mpu.getAccelOffsetY());
    Serial.print(" / ");
    Serial.println(mpu.getAccelOffsetZ());

    Serial.print(" * Accelerometer power delay: ");
    switch (mpu.getAccelPowerOnDelay())
    {
    case MPU6050_DELAY_3MS:
        Serial.println("3ms");
        break;
    case MPU6050_DELAY_2MS:
        Serial.println("2ms");
        break;
    case MPU6050_DELAY_1MS:
        Serial.println("1ms");
        break;
    case MPU6050_NO_DELAY:
        Serial.println("0ms");
        break;
    }

    Serial.println();
}

void readaccel()
{
}

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
        Vector rawAccel = mpu.readNormalizeAccel();
        Activites act = mpu.readActivites();
        if (rawAccel.XAxis >= 0)
        {
            analogWrite(LED_FORWARD_PIN, (int)rawAccel.XAxis);
            analogWrite(LED_BACKWARDS_PIN, 0);
        }
        else
        {
            analogWrite(LED_BACKWARDS_PIN, (int)rawAccel.XAxis * -1);
            analogWrite(LED_FORWARD_PIN, 0);
        }

        COROUTINE_DELAY(50);
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

    accelLEDHandler.runCoroutine();

    // thresholdStepBtn.update();
    // if (thresholdStepBtn.transitioned_to(LOW))
    // {
    //     accel_threshold_step();
    // }

    // mpuInterrupt = false;

    led_alive_state = !led_alive_state;
    digitalWrite(LED_ALIVE_PIN, led_alive_state);
    delay(100);
}
