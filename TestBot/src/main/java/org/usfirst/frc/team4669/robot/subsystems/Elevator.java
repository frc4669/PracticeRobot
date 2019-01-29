package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.TeleopClimber;
// import org.usfirst.frc.team4669.robot.commands.TeleopElevator;
import org.usfirst.frc.team4669.robot.misc.Constants;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
*
*/
public class Elevator extends Subsystem {

    private WPI_TalonSRX leftMotor;
    private WPI_TalonSRX rightMotor;
    private Accelerometer accel;

    private int timeout = Constants.timeout;
    private int slotIdx = RobotMap.slotIdx;
    private int pidIdx = RobotMap.pidIdx;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public Elevator() {
        super();
        leftMotor = new WPI_TalonSRX(RobotMap.leftMotorElevator);
        rightMotor = new WPI_TalonSRX(RobotMap.rightMotorElevator);

        accel = new BuiltInAccelerometer();

        setupMotor(leftMotor, false);
        setupMotor(rightMotor, true);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new TeleopClimber());
    }

    public void percentOutputLeft(double percent) {
        leftMotor.set(ControlMode.PercentOutput, percent);
    }

    public void percentOutputRight(double percent) {
        rightMotor.set(ControlMode.PercentOutput, percent);
    }

    // public void setHeight(double height) {
    // elevatorMotor.configMotionCruiseVelocity(Constants.elevatorVel,
    // Constants.timeout);
    // elevatorMotor.configMotionAcceleration(Constants.elevatorAccel,
    // Constants.timeout);
    // elevatorMotor.set(ControlMode.MotionMagic, height);
    // }

    // public void controlHeight(double power, double initialPos) {
    // elevatorMotor.configMotionCruiseVelocity(400, Constants.timeout);
    // elevatorMotor.configMotionAcceleration(400, Constants.timeout);
    // double targetPos = initialPos;
    // double distanceToTravel = (int) (3 * 4096 / (Math.PI * 2)); // 3 inches max,2
    // inch sprocket diameter
    // targetPos += power * distanceToTravel;
    // elevatorMotor.set(ControlMode.MotionMagic, targetPos);
    // }

    // public void groundHeight() {
    // elevatorMotor.configMotionCruiseVelocity(Constants.elevatorDownVel,
    // Constants.timeout);
    // elevatorMotor.configMotionAcceleration(Constants.elevatorDownAccel,
    // Constants.timeout);
    // elevatorMotor.set(ControlMode.MotionMagic, 0);

    // }

    // public double getClosedLoopError() {
    // return elevatorMotor.getClosedLoopError(0);
    // }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void setupMotor(TalonSRX talon, boolean invert) {
        talon.configFactoryDefault();
        talon.configNominalOutputForward(0, timeout);
        talon.configNominalOutputReverse(0, timeout);
        talon.configPeakOutputForward(1, timeout);
        talon.configPeakOutputReverse(-1, timeout);

        // Sets current limits
        talon.configContinuousCurrentLimit(12, timeout);
        talon.configPeakCurrentLimit(15, timeout);
        talon.configPeakCurrentDuration(100, timeout);
        talon.enableCurrentLimit(true);

        // Sets up sensor
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidIdx, timeout);
        talon.setSensorPhase(false);
        talon.setInverted(invert);
        talon.setSelectedSensorPosition(0, pidIdx, timeout);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.timeout);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.timeout);

        // Configuring PID Values
        talon.selectProfileSlot(slotIdx, pidIdx);
        talon.config_kF(slotIdx, Constants.elevatorPID[0], timeout);
        talon.config_kP(slotIdx, Constants.elevatorPID[1], timeout);
        talon.config_kI(slotIdx, Constants.elevatorPID[2], timeout);
        talon.config_kD(slotIdx, Constants.elevatorPID[3], timeout);
        talon.config_IntegralZone(slotIdx, (int) Constants.elevatorPID[4], timeout);

        // Set relevant frame periods to be at least as fast as periodic rate
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout);

        // Sets up velocity and acceleration for motion magic
        talon.configMotionCruiseVelocity(Constants.elevatorVel, Constants.timeout);
        talon.configMotionAcceleration(Constants.elevatorAccel, Constants.timeout);

        // Zero encoders on fwd limit or bottom limit
        talon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 10);

        // Sets reverse soft limit
        // talon.configReverseSoftLimitThreshold(RobotMap.elevatorMax,
        // RobotMap.timeout);
        talon.configReverseSoftLimitEnable(false, Constants.timeout);

        // //Sets forward soft limit
        // talon.configForwardSoftLimitThreshold(0, RobotMap.timeout);
        talon.configForwardSoftLimitEnable(false, Constants.timeout);
    }

    public void setMotionMagic(TalonSRX talon, double position) {
        talon.set(ControlMode.MotionMagic, position);
    }

    public double getEncoderPos(TalonSRX talon) {
        return talon.getSelectedSensorPosition();
    }

    public double getEncoderVel(TalonSRX talon) {
        return talon.getSelectedSensorVelocity();
    }

    public double getMotorOutput(TalonSRX talon) {
        return talon.getMotorOutputPercent();
    }

    public void zeroEncoders() {
        leftMotor.setSelectedSensorPosition(0, pidIdx, timeout);
        rightMotor.setSelectedSensorPosition(0, pidIdx, timeout);
    }

    public boolean getForwardLimit(TalonSRX talon) {
        return talon.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimit(TalonSRX talon) {
        return talon.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void zeroVelocity(TalonSRX talon) {
        talon.set(ControlMode.Velocity, 0);
    }

    public double getAccelX() {
        return accel.getX();
    }

    public double getAccelY() {
        return accel.getY();
    }

    public TalonSRX getLeftMotor() {
        return leftMotor;
    }

    public TalonSRX getRightMotor() {
        return rightMotor;
    }

    public void setVelocity(TalonSRX talon, double velocity) {
        talon.set(ControlMode.Velocity, velocity);
    }
}
