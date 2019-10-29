#include "frc/Joystick.h"
#include "frc/buttons/JoystickButton.h"
#include "frc/buttons/POVButton.h"
#include "RobotMap.h"

class F310
{

public:

    frc::Joystick f310joystick{RobotMap::f310};

    const int green_button = 1;
    const int red_button = 2;
    const int blue_button = 3;
    const int orange_button = 4;
    const int left_shoulder_button = 5;
    const int right_shoulder_button = 6;
    const int back_button = 7;
    const int start_button = 8;
    const int left_joy_button = 9;
    const int right_joy_button = 10;

    frc::JoystickButton greenButtonObject{&f310joystick, green_button};
    frc::JoystickButton redButtonObject{&f310joystick, red_button};
	frc::JoystickButton blueButtonObject{&f310joystick, blue_button};
	frc::JoystickButton orangeButtonObject{&f310joystick, orange_button};
	frc::JoystickButton leftShoulderButtonObject{&f310joystick, left_shoulder_button};
	frc::JoystickButton rightShoulderButtonObject{&f310joystick, right_shoulder_button};
	frc::JoystickButton backButtonObject{&f310joystick, back_button};
	frc::JoystickButton startButtonObject{&f310joystick, start_button};
	frc::JoystickButton leftJoyButtonObject{&f310joystick, left_joy_button};
	frc::JoystickButton rightJoyButtonObject{&f310joystick, right_joy_button};

    F310();
    double getLeftX();
    double getLeftXCubed();
    double getLeftY();
    double getLeftYCubed();
    double getRightX();
    double getRightXCubed();
    double getRightY();
    double getRightYCubed();
    double getLeftTrigger();
    double getRightTrigger();
    bool getButton(int buttonPort);
    bool getButtonPressed(int buttonPort);
    bool getButtonReleased(int buttonPort);
    int getDPadPOV();
    double deadzone(int port);

};
