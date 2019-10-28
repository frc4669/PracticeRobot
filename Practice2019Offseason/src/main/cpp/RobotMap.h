namespace RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;

	/** CAN IDs for motor controllers */
    
        const int driveFrontLeft = 5;
        const int driveRearLeft = 12;
        const int driveFrontRight = 11;
        const int driveRearRight = 9;

        const int leftMotorElevator = 1;
        const int rightMotorElevator = 7; // 2
        const int wheelMotorElevator = 3;

        const int shoulderMotor = 13;
        const int wristMotor = 2;
        const int elbowMotor = 6; // 2

        /** USB IDs for gamepad and joysticks */
        const int leftJoystick = 0;
        const int rightJoystick = 1;
        const int f310 = 2;
        const int extremeJoystick = 3;
        const int buttonBoard = 4;
        const int testButonBoard = 5;


        /** Talon SRX Encoder Slot and PID Slot */
        const int slotIdx = 0;
        const int pidIdx = 0;

        /** Sensor IDs */
        const int frontUltrasonicEcho = 4;
        const int frontUltrasonicTrigger = 5;
        const int rearUltrasonicEcho = 2;
        const int rearUltrasonicTrigger = 3;
        const int rightInfrared = 0; // Infrared sensors use analog channels;
        const int midInfrared = 1;
        const int leftInfrared = 2;

        /** Pneumatic Channels */
        const int solenoidForward = 0;
        const int solenoidReverse = 1;

}