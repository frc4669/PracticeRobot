#include "frc/Joystick.h"
#include "frc/buttons/JoystickButton.h"
#include "frc/buttons/POVButton.h"

class F310
{

public:
    frc::Joystick * f310joystick;

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
    
    frc::JoystickButton * greenButtonObject;
    frc::JoystickButton * redButtonObject;
	frc::JoystickButton * blueButtonObject;
	frc::JoystickButton * orangeButtonObject;
	frc::JoystickButton * leftShoulderButtonObject;
	frc::JoystickButton * rightShoulderButtonObject;
	frc::JoystickButton * backButtonObject;
	frc::JoystickButton * startButtonObject;
	frc::JoystickButton * leftJoyButtonObject;
	frc::JoystickButton * rightJoyButtonObject;

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
