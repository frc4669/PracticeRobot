#include "frc/Joystick.h"
#include "frc/buttons/JoystickButton.h"
#include "frc/buttons/POVButton.h"
#include "F310.h"
#include "RobotMap.h"

F310::F310() 
{
    f310joystick = new frc::Joystick(RobotMap::f310);

    greenButtonObject = new frc::JoystickButton(f310joystick, F310::green_button);
    redButtonObject = new frc::JoystickButton(f310joystick, F310::red_button);
    blueButtonObject = new frc::JoystickButton(f310joystick, F310::blue_button);
    orangeButtonObject = new frc::JoystickButton(f310joystick, F310::orange_button);
    leftShoulderButtonObject = new frc::JoystickButton(f310joystick, F310::left_shoulder_button);
    rightShoulderButtonObject = new frc::JoystickButton(f310joystick, F310::right_shoulder_button);
    backButtonObject = new frc::JoystickButton(f310joystick, F310::back_button);
    startButtonObject = new frc::JoystickButton(f310joystick, F310::start_button);
    leftJoyButtonObject = new frc::JoystickButton(f310joystick, F310::left_joy_button);
    rightJoyButtonObject = new frc::JoystickButton(f310joystick, F310::right_joy_button);
    // startButtonObject::whenPressed(new StopAll());
    // blueButtonObject::whenPressed(new ToggleCompressor());

}

double F310::getLeftX() {
    return deadzone(0);
}

double F310::getLeftXCubed() {
    return pow(getLeftX(), 3);
}

double F310::getLeftY() {
    return -deadzone(1); // Negates the Y axis of the joystick so that up is positive and down is
                            // negative
}

double F310::getLeftYCubed() {
    return pow(getLeftY(), 3);
}

double F310::getLeftTrigger() {
    return deadzone(2);
}

double F310::getRightTrigger() {
    return deadzone(3);
}

double F310::getRightX() {
    return deadzone(4);
}

double F310::getRightXCubed() {
    return pow(getRightX(), 3);
}

// ControlWinch
double F310::getRightY() {
    return -deadzone(5); // Negates the Y axis of the joystick so that up is positive and down is
                            // negative
}

bool F310::getButton(int buttonPort) {
    return f310joystick->GetRawButton(buttonPort);
}

bool F310::getButtonPressed(int buttonPort) {
    return f310joystick->GetRawButtonPressed(buttonPort);
}

bool F310::getButtonReleased(int buttonPort) {
    return f310joystick->GetRawButtonReleased(buttonPort);
}

int F310::getDPadPOV() {
    return f310joystick->GetPOV();
}

double F310::deadzone(int port) {
    double joystickValue = f310joystick->GetRawAxis(port);
    double joystickOffset = 0.075;
    double absJoystickValue = abs(joystickValue);
    if (absJoystickValue > joystickOffset) {
        double speed = absJoystickValue;
        speed = (speed * speed) + joystickOffset;
        if (joystickValue > 0)
            return speed;
        else
            return -speed;
    } else {
        return 0;
    }
}