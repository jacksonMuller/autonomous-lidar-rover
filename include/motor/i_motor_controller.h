#pragma once

class IMotorController {
public:
    virtual ~IMotorController() = default;

    virtual void forward(float speed) = 0;
    virtual void turnLeft(float speed) = 0;
    virtual void turnRight(float speed) = 0;
    virtual void stop() = 0;
};