// package org.usfirst.frc.team4669.robot.subsystems;

// import org.usfirst.frc.team4669.robot.Robot;
// import org.usfirst.frc.team4669.robot.RobotMap;
// import org.usfirst.frc.team4669.robot.commands.TeleopIntake;
// import org.usfirst.frc.team4669.robot.misc.Constants;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Ultrasonic;

// /**
// *
// */
// public class CubeIntake extends Subsystem {

// private WPI_TalonSRX rightIntakeMotor;
// private WPI_TalonSRX leftIntakeMotor;
// private Servo ropeServo;
// private AnalogInput distSensorLeft;
// private AnalogInput distSensorRight;

// private int timeout = Constants.timeout;
// private int slotIdx = RobotMap.slotIdx;
// private int pidIdx = RobotMap.pidIdx;

// // Put methods for controlling this subsystem
// // here. Call these from Commands.

// public CubeIntake() {
// super();

// distSensorLeft = new AnalogInput(0);
// distSensorRight = new AnalogInput(1);
// ropeServo = new Servo(0);

// rightIntakeMotor = new WPI_TalonSRX(RobotMap.leftIntake);
// leftIntakeMotor = new WPI_TalonSRX(RobotMap.rightIntake);

// rightIntakeMotor.configNominalOutputForward(0, timeout);
// rightIntakeMotor.configNominalOutputReverse(0, timeout);
// leftIntakeMotor.configNominalOutputForward(0, timeout);
// leftIntakeMotor.configNominalOutputReverse(0, timeout);
// rightIntakeMotor.configPeakOutputForward(1, timeout);
// rightIntakeMotor.configPeakOutputReverse(-1, timeout);
// leftIntakeMotor.configPeakOutputForward(1, timeout);
// leftIntakeMotor.configPeakOutputReverse(-1, timeout);

// rightIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
// pidIdx, timeout);
// leftIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
// pidIdx, timeout);

// rightIntakeMotor.setSelectedSensorPosition(0, pidIdx, timeout);
// leftIntakeMotor.setSelectedSensorPosition(0, pidIdx, timeout);

// rightIntakeMotor.setSensorPhase(true);
// leftIntakeMotor.setSensorPhase(false);

// rightIntakeMotor.setInverted(false);
// leftIntakeMotor.setInverted(true);

// // Setting Current limits

// rightIntakeMotor.configContinuousCurrentLimit(12, timeout);
// rightIntakeMotor.configPeakCurrentLimit(15, timeout);
// rightIntakeMotor.configPeakCurrentDuration(100, timeout);
// rightIntakeMotor.enableCurrentLimit(true);

// leftIntakeMotor.configContinuousCurrentLimit(12, timeout);
// leftIntakeMotor.configPeakCurrentLimit(15, timeout);
// leftIntakeMotor.configPeakCurrentDuration(100, timeout);
// leftIntakeMotor.enableCurrentLimit(true);
// }

// public void initDefaultCommand() {
// // Set the default command for a subsystem here.
// // setDefaultCommand(new MySpecialCommand());
// setDefaultCommand(new TeleopIntake());
// }

// public void intake() {
// if (!hasCube())
// set(0.6, 0.6);
// }

// public void releaseCube() {
// set(-0.3, -0.3);
// }

// public void turnCubeRight() {
// rightIntakeMotor.set(ControlMode.PercentOutput, 0.7);
// }

// public void turnCubeLeft() {
// leftIntakeMotor.set(ControlMode.PercentOutput, 0.7);
// }

// public void set(double outputL, double outputR) {
// leftIntakeMotor.set(ControlMode.PercentOutput, outputL);
// rightIntakeMotor.set(ControlMode.PercentOutput, outputR);
// }

// public void stopIntake() {
// set(0, 0);
// }

// public void stopLeft() {
// leftIntakeMotor.set(ControlMode.PercentOutput, 0);
// }

// public void stopRight() {
// rightIntakeMotor.set(ControlMode.PercentOutput, 0);
// }

// public void releaseArms() {
// ropeServo.set(0);// 0 means rotate full counter-clockwise, 1 means full
// clockwise
// }

// public void stopServo() {
// ropeServo.set(0.5);// 0.5 means stop
// }

// // Getting encoder velocities and position, distance sensor range

// public double getLeftEncoder() {
// return rightIntakeMotor.getSensorCollection().getQuadraturePosition();
// }

// public double getRightEncoder() {
// return leftIntakeMotor.getSensorCollection().getQuadraturePosition();
// }

// public double getLeftEncoderSpeed() {
// return rightIntakeMotor.getSensorCollection().getQuadratureVelocity();
// }

// public double getRightEncoderSpeed() {
// return leftIntakeMotor.getSensorCollection().getQuadratureVelocity();
// }

// public double getLeftDistance() {
// return distSensorLeft.getVoltage(); // reads the range on the distance sensor
// in voltage, 0 is farther
// }

// public double getRightDistance() {
// return distSensorRight.getVoltage(); // reads the range on the distance
// sensor in voltage, 0 is farther
// }

// public boolean hasCube() {
// return getLeftDistance() > 1.2 && getRightDistance() > 1.2;
// }
// }
