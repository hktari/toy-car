#include "../src/toy-car.ino"
#include "gtest/gtest.h"
#include "arduino-mock/Arduino.h"

using ::testing::Return;

class ToyCar : public testing::Test
{
protected:
    void SetUp() override
    {
        car_state = CarState::OFF;
        ArduinoMock *arduinoMock = arduinoMockInstance();
        arduinoMock->digitalWrite(START_PIN, LOW);
        arduinoMock->digitalWrite(FRONT_LIGHTS_PINS[0], LOW);
        arduinoMock->digitalWrite(FRONT_LIGHTS_PINS[1], LOW);
        // setup();
    }

    // void TearDown() override{}
    // {
    // }
};

TEST_F(ToyCar, TurnsOn)
{
    // ArduinoMock *arduinoMock = arduinoMockInstance();
    // arduinoMock->digitalWrite(START_PIN, HIGH);
    // loop();
    // EXPECT_EQ(car_state, CarState::RUNNING) << "Car state should be 'RUNNING'";
    // releaseArduinoMock();
}

// int main()
// {
//     setup();
//     loop();
// }
