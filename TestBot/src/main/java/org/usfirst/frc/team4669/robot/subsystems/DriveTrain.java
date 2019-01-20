package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.TeleopDrive;
import org.usfirst.frc.team4669.robot.misc.VisionPIDSource.BallAlign;
import org.usfirst.frc.team4669.robot.misc.Constants;
import org.usfirst.frc.team4669.robot.misc.PIDOutputWrapper;
import org.usfirst.frc.team4669.robot.misc.VisionPIDSource;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 *
 */
public class DriveTrain extends Subsystem {

	private WPI_TalonSRX topLeftMotor;
	private WPI_TalonSRX bottomLeftMotor;
	private WPI_TalonSRX topRightMotor;
	private WPI_TalonSRX bottomRightMotor;

	private SpeedControllerGroup leftMotorGroup;
	private SpeedControllerGroup rightMotorGroup;
	private DifferentialDrive drive;

	public VisionPIDSource visionDistance;
	public VisionPIDSource visionTurn;

	public PIDController gyroPID;
	public PIDController visionTurnController;
	public PIDController visionDistanceController;

	private PIDOutputWrapper turnOutput;
	private PIDOutputWrapper visionTurnOutput;
	private PIDOutputWrapper visionDistanceOutput;

	int velocity = 2300; // About 200 RPM, vel units are in sensor units per 100ms
	int accel = 4600;

	private Gyro gyro;

	public DriveTrain() {
		super();
		gyro = new ADXRS450_Gyro();

		visionDistance = new VisionPIDSource(BallAlign.DISTANCE);
		visionTurn = new VisionPIDSource(BallAlign.TURN);

		topLeftMotor = new WPI_TalonSRX(RobotMap.driveTopLeft);
		bottomLeftMotor = new WPI_TalonSRX(RobotMap.driveBottomLeft);
		topRightMotor = new WPI_TalonSRX(RobotMap.driveTopRight);
		bottomRightMotor = new WPI_TalonSRX(RobotMap.driveBottomRight);

		// Configuring the Gyroscope and the PID controller for it
		turnOutput = new PIDOutputWrapper();
		gyroPID = new PIDController(Constants.gyroPID[0], Constants.gyroPID[1], Constants.gyroPID[2], (PIDSource) gyro,
				turnOutput);
		gyroPID.setInputRange(0, 360);
		gyroPID.setContinuous(true);
		gyroPID.setOutputRange(-0.5, 0.5);
		gyroPID.setAbsoluteTolerance(3);

		// Configuring the Vision PID controllers
		visionTurnOutput = new PIDOutputWrapper();
		visionTurnController = new PIDController(Constants.cameraPID[0], Constants.cameraPID[1], Constants.cameraPID[2],
				visionTurn, visionTurnOutput);
		visionTurnController.setInputRange(0, 320);
		visionTurnController.setContinuous(false);
		visionTurnController.setOutputRange(-0.5, 0.5);
		visionTurnController.setAbsoluteTolerance(10);

		visionDistanceOutput = new PIDOutputWrapper();
		visionDistanceController = new PIDController(Constants.cameraPID[0], Constants.cameraPID[1],
				Constants.cameraPID[2], visionDistance, visionDistanceOutput);
		visionDistanceController.setInputRange(0, 200);
		visionDistanceController.setContinuous(false);
		visionDistanceController.setOutputRange(-0.5, 0.5);
		visionDistanceController.setAbsoluteTolerance(10);

		talonConfig(topRightMotor, false);
		talonConfig(topLeftMotor, true);
		bottomRightMotor.setInverted(true);
		bottomLeftMotor.setInverted(false);

		// Set Current limit
		setCurrentLimit(topRightMotor);
		setCurrentLimit(topLeftMotor);
		setCurrentLimit(bottomRightMotor);
		setCurrentLimit(bottomLeftMotor);

		/* set acceleration and vcruise velocity - see documentation */
		setMotionVelAccel(velocity, accel);
		topRightMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
		topLeftMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);

		// Setting bottom motors to follow top
		bottomRightMotor.set(ControlMode.Follower, topRightMotor.getDeviceID());
		bottomLeftMotor.set(ControlMode.Follower, topLeftMotor.getDeviceID());

		leftMotorGroup = new SpeedControllerGroup(topLeftMotor, bottomLeftMotor);
		rightMotorGroup = new SpeedControllerGroup(topRightMotor, bottomRightMotor);

		leftMotorGroup.setInverted(false);
		rightMotorGroup.setInverted(false);

		drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
		drive.setRightSideInverted(false);

	}

	public void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}

	// Drive methods

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}

	public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
		drive.arcadeDrive(xSpeed, zRotation, squaredInputs);
	}

	public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
		drive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
	}

	public void stop() {
		topLeftMotor.set(ControlMode.PercentOutput, 0);
		topRightMotor.set(ControlMode.PercentOutput, 0);
	}

	public void driveMotionMagic(double targetEncPosition) {
		setMotionVelAccel(this.velocity, this.accel);
		topLeftMotor.set(ControlMode.MotionMagic, targetEncPosition);
		topRightMotor.set(ControlMode.MotionMagic, targetEncPosition);
	}

	public void setMotionVelAccel(int velocity, int accel) {
		topLeftMotor.configMotionCruiseVelocity(velocity, Constants.timeout);
		topLeftMotor.configMotionAcceleration(accel, Constants.timeout);
		topRightMotor.configMotionCruiseVelocity(velocity, Constants.timeout);
		topRightMotor.configMotionAcceleration(accel, Constants.timeout);
	}

	public void turnMagic(double angle) {
		setMotionVelAccel(1365, 6825);
		double d = -(Constants.wheelBase * Math.PI * (angle / 360.0) * Constants.inchToEncoder);
		topLeftMotor.set(ControlMode.MotionMagic, -d);
		topRightMotor.set(ControlMode.MotionMagic, d);
		System.out.println(d);
	}

	public void turnTo(boolean clockwise) {
		// If it is negative, turn clockwise
		if (clockwise) {
			tankDrive(0.5, -0.5, false);
		}
		// If it is positive, turn counterclockwise
		else {
			tankDrive(-0.5, 0.5, false);
		}
	}

	// Methods to control and measure sensors such as gyro and encoders

	public void calibrateGyro() {
		gyro.calibrate();
	}

	public void resetGyro() {
		gyro.reset();
	}

	public int getLeftEncoder() {
		return topLeftMotor.getSelectedSensorPosition();
	}

	public int getRightEncoder() {
		return topRightMotor.getSelectedSensorPosition();
	}

	public double getLeftEncoderSpeed() {
		return topLeftMotor.getSelectedSensorVelocity();
	}

	public double getRightEncoderSpeed() {
		return topRightMotor.getSelectedSensorVelocity();
	}

	public void zeroEncoders() {
		topLeftMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
		topRightMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
	}

	public double getLeftCurrent() {
		return topLeftMotor.getOutputCurrent();
	}

	public double getRightCurrent() {
		return topRightMotor.getOutputCurrent();
	}

	public void enablePIDController(PIDController controller) {
		controller.reset();
		controller.enable();
	}

	public void disablePIDController(PIDController controller) {
		controller.disable();
	}

	public void setTarget(PIDController controller, double target) {
		controller.setSetpoint(target);
	}

	public boolean getPIDDone(PIDController controller) {
		return controller.onTarget();
	}

	public double getPIDError(PIDController controller) {
		return controller.getError();
	}

	public double getTurnOutput() {
		return turnOutput.getOutput();
	}

	public double getVisionTurnOutput() {
		return visionTurnOutput.getOutput();
	}

	public double getVisionDistanceOutput() {
		return visionDistanceOutput.getOutput();
	}

	public boolean isObjectDetected() {
		return visionDistance.isObjectDetected();
	}

	public void setMode(ControlMode mode, double value) {
		topLeftMotor.set(mode, value);
		topRightMotor.set(mode, value);
	}

	public double getAngle() {
		return gyro.getAngle();
	}

	public PIDController getGyroController() {
		return gyroPID;
	}

	public PIDController getVisionTurnController() {
		return visionTurnController;
	}

	public PIDController getVisionDistanceController() {
		return visionDistanceController;
	}

	// Method needed to set up status frame period for Talon SRX motion profile
	public void setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
		Robot.driveTrain.topLeftMotor.setStatusFramePeriod(frame, periodMs, timeoutMs);
		Robot.driveTrain.topRightMotor.setStatusFramePeriod(frame, periodMs, timeoutMs);
	}

	public WPI_TalonSRX getTopLeftMotor() {
		return topLeftMotor;
	}

	public WPI_TalonSRX getTopRightMotor() {
		return topRightMotor;
	}

	public void driveStraightGyro(double power, double targetedAngle, double kP) {
		double error = getAngle() - targetedAngle;
		double turnPower = error * kP;
		arcadeDrive(power, turnPower, false);
	}

	public void followPath(Notifier followerNotifier, EncoderFollower leftFollower, EncoderFollower rightFollower) {
		if (leftFollower.isFinished() || rightFollower.isFinished()) {
			followerNotifier.stop();
		} else {
			double left_speed = leftFollower.calculate(Robot.driveTrain.getLeftEncoder());
			double right_speed = rightFollower.calculate(Robot.driveTrain.getRightEncoder());
			double heading = Robot.driveTrain.getAngle();
			double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
			double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
			double turn = 0.8 * (-1.0 / 80.0) * heading_difference;
			System.out.println("Turn: " + turn);
			System.out.println("Left speed: " + left_speed + " Combined: " + (left_speed + turn));
			System.out.println("Right speed: " + right_speed + " Combined: " + (right_speed + turn));
			topLeftMotor.set(left_speed + turn);
			topRightMotor.set(right_speed - turn);
		}
	}

	public void talonConfig(TalonSRX talon, boolean left) {
		talon.configFactoryDefault();

		talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidIdx, Constants.timeout);
		talon.setInverted(!left);
		talon.setSensorPhase(true);

		talon.selectProfileSlot(RobotMap.slotIdx, RobotMap.pidIdx);
		talon.config_kF(RobotMap.slotIdx, Constants.driveTrainPID[0], Constants.timeout);
		talon.config_kP(RobotMap.slotIdx, Constants.driveTrainPID[1], Constants.timeout);
		talon.config_kI(RobotMap.slotIdx, Constants.driveTrainPID[2], Constants.timeout);
		talon.config_kD(RobotMap.slotIdx, Constants.driveTrainPID[3], Constants.timeout);
		talon.config_IntegralZone(RobotMap.slotIdx, (int) Constants.driveTrainPID[4], Constants.timeout);

		talon.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
	}

	public void setCurrentLimit(TalonSRX talon) {
		talon.configContinuousCurrentLimit(Constants.continuousCurrentLimit, Constants.timeout);
		talon.configPeakCurrentLimit(Constants.peakCurrentLimit, Constants.timeout);
		talon.configPeakCurrentDuration(Constants.currentDuration, Constants.timeout);
		talon.enableCurrentLimit(true);
	}

}
