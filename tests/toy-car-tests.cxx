#include "../src/toy-car.ino"
#include "gtest/gtest.h"
#include "arduino-mock/Arduino.h"

using ::testing::Return;
using ::testing::Sequence;

class ToyCar : public testing::Test
{
protected:
    void SetUp() override
    {
        car_state = CarState::OFF;
        last_move_timestamp = 0;

        // ArduinoMock *arduinoMock = arduinoMockInstance();
        // digitalWrite(START_PIN, LOW);
        // digitalWrite(FRONT_LIGHTS_PINS[0], LOW);
        // digitalWrite(FRONT_LIGHTS_PINS[1], LOW);
        setup();

        releaseArduinoMock();
    }

    void TearDown() override
    {
        releaseArduinoMock();
    }
};

TEST_F(ToyCar, TurnsOn)
{
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalRead(START_PIN)).WillOnce(Return(HIGH));
    EXPECT_CALL(*arduinoMock, digitalWrite(FRONT_LIGHTS_PINS[0], HIGH));
    EXPECT_CALL(*arduinoMock, digitalWrite(FRONT_LIGHTS_PINS[1], HIGH));

    loop();

    EXPECT_EQ(car_state, CarState::RUNNING) << "Car state should be 'RUNNING'";

    releaseArduinoMock();
}

// int main()
// {
//     setup();
//     loop();
// }
