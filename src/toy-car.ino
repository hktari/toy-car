#ifdef TEST
#pragma once
#include "arduino-mock/Arduino.h"
#endif

#include <AceRoutine.h>
using namespace ace_routine;

#include <math.h>
#include <Wire.h>
#include <MPU6050.h>
#include <avr/sleep.h>

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

CarState car_state = CarState::OFF;

unsigned long last_move_timestamp = 0;

const int LEDS_TOP_BLINK_SPEED = 250;
#define LEDS_TOP_BLUE 8
#define LEDS_TOP_RED 7
#define LEDS_FRONT 6

#define LED_FORWARD_PIN 3
#define LED_BACKWARDS_PIN 5
#define THRESHOLD_STEP_BTN_PIN 4

#define START_PIN 2      // use pin 2 on Arduino Uno & most boards
#define LED_ALIVE_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool led_alive_state = false;

Button start_btn(START_PIN);

void setup()
{
    Serial.begin(115200);

    pinMode(LED_ALIVE_PIN, OUTPUT);
    pinMode(LED_FORWARD_PIN, OUTPUT);
    pinMode(LED_BACKWARDS_PIN, OUTPUT);
    pinMode(THRESHOLD_STEP_BTN_PIN, INPUT_PULLUP);

    pinMode(LEDS_FRONT, OUTPUT);
    pinMode(START_PIN, INPUT_PULLUP);
    pinMode(LEDS_TOP_BLUE, OUTPUT);
    pinMode(LEDS_TOP_RED, OUTPUT);

    digitalWrite(LEDS_FRONT, LOW);

    setup_mpu6050();

    delay(500);

    car_state = CarState::RUNNING;
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

float accel_threshold = 1.5;
Button thresholdStepBtn(THRESHOLD_STEP_BTN_PIN);

void accel_threshold_step()
{
    accel_threshold += 0.5;
    Serial.print("Accel threshold: ");
    Serial.print(accel_threshold, DEC);
    Serial.println();
}

Vector curAccel;
Activites curActvt;

COROUTINE(accelLEDHandler)
{
    COROUTINE_LOOP()
    {
        curAccel = mpu.readNormalizeAccel();
        curActvt = mpu.readActivites();

        if (curActvt.isActivity)
        {
            last_move_timestamp = millis();
            Serial.println("Moved");
        }

        if (curAccel.XAxis >= 0)
        {
            analogWrite(LED_FORWARD_PIN, (int)curAccel.XAxis);
            analogWrite(LED_BACKWARDS_PIN, 0);
        }
        else
        {
            analogWrite(LED_BACKWARDS_PIN, (int)curAccel.XAxis * -1);
            analogWrite(LED_FORWARD_PIN, 0);
        }

        COROUTINE_DELAY(50);
    }
}

COROUTINE(topLEDHandler)
{
    COROUTINE_LOOP()
    {
        analogWrite(LEDS_TOP_BLUE, HIGH);
        analogWrite(LEDS_TOP_RED, LOW);
        COROUTINE_DELAY(LEDS_TOP_BLINK_SPEED);
        analogWrite(LEDS_TOP_BLUE, LOW);
        analogWrite(LEDS_TOP_RED, HIGH);
    }
}

void on_wake()
{
}
void go_to_sleep()
{
    Serial.println("Going to sleep..");
    Serial.flush();
    delay(10);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    attachInterrupt(digitalPinToInterrupt(START_PIN), on_wake, LOW);

    // Do not interrupt before we go to sleep, or the
    // ISR will detach interrupts and we won't wake.
    noInterrupts();

    // We are guaranteed that the sleep_cpu call will be done
    // as the processor executes the next instruction after
    // interrupts are turned on.
    interrupts(); // one cycle
    sleep_cpu();  // one cycle

    // first thing after waking from sleep:
    detachInterrupt(digitalPinToInterrupt(START_PIN));

    start_car();
}

void start_car()
{
    digitalWrite(LEDS_FRONT, HIGH);
    car_state = CarState::RUNNING;
    // TODO: play sfx
}

void loop()
{
    if (car_state == CarState::RUNNING)
    {
        time_t time = millis();
        if (time - last_move_timestamp >= TURN_OFF_DELAY)
        {
            car_state = CarState::OFF;

            digitalWrite(LEDS_FRONT, LOW);
            digitalWrite(LEDS_TOP_BLUE, LOW);
            digitalWrite(LEDS_TOP_RED, LOW);

            go_to_sleep();
        }

        accelLEDHandler.runCoroutine();
        topLEDHandler.runCoroutine();
    }

    led_alive_state = !led_alive_state;
    digitalWrite(LED_ALIVE_PIN, led_alive_state);
    delay(100);
}
