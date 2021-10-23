#include "../src/toy-car.ino"
#include "gtest/gtest.h"
#include "arduino-mock/Arduino.h"

using ::testing::Return;
using ::testing::Sequence;

class RunningToyCar : public testing::Test
{
protected:
    void SetUp() override
    {
        setup();
        car_state = CarState::RUNNING;
        last_move_timestamp = 0;
    }

    void TearDown() override
    {
        releaseArduinoMock();
    }
};

TEST_F(RunningToyCar, TurnsOffAfterDelay)
{
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_EQ(car_state, CarState::RUNNING) << "Car state should begin in 'RUNNING' state";

    EXPECT_CALL(*arduinoMock, digitalRead(START_PIN)).WillRepeatedly(Return(LOW));

    {
        Sequence sq;

        EXPECT_CALL(*arduinoMock, digitalWrite(FRONT_LIGHTS_PINS[0], LOW));
        EXPECT_CALL(*arduinoMock, digitalWrite(FRONT_LIGHTS_PINS[1], LOW));

        // TODO: fork arduino mock repo and change millis() implementation so this mock works 
        EXPECT_CALL(*arduinoMock, millis()).WillOnce(Return(0)).WillOnce(Return(TURN_OFF_DELAY));

        loop();
        loop();

        EXPECT_EQ(car_state, CarState::OFF) << "Car state should be 'OFF'";
    }
    releaseArduinoMock();
}
