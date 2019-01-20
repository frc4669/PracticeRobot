package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.TeleopElevator;
import org.usfirst.frc.team4669.robot.misc.Constants;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
*
*/
public class Elevator extends Subsystem {

    private WPI_TalonSRX elevatorMotor;

    private int timeout = Constants.timeout;
    private int slotIdx = RobotMap.slotIdx;
    private int pidIdx = RobotMap.pidIdx;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public Elevator() {
        super();
        elevatorMotor = new WPI_TalonSRX(RobotMap.elevator);

        // Set relevant frame periods to be at least as fast as periodic rate
        elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout);
        elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout);

        elevatorMotor.configNominalOutputForward(0, timeout);
        elevatorMotor.configNominalOutputReverse(0, timeout);
        elevatorMotor.configPeakOutputForward(1, timeout);
        elevatorMotor.configPeakOutputReverse(-1, timeout);

        // Configuring PID Values
        elevatorMotor.selectProfileSlot(slotIdx, pidIdx);
        elevatorMotor.config_kF(slotIdx, Constants.elevatorPID[0], timeout);
        elevatorMotor.config_kP(slotIdx, Constants.elevatorPID[1], timeout);
        elevatorMotor.config_kI(slotIdx, Constants.elevatorPID[2], timeout);
        elevatorMotor.config_kD(slotIdx, Constants.elevatorPID[3], timeout);
        elevatorMotor.config_IntegralZone(slotIdx, (int) Constants.elevatorPID[4], timeout);

        // Sets up sensor
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidIdx, timeout);
        elevatorMotor.setSelectedSensorPosition(0, pidIdx, timeout);
        elevatorMotor.setSensorPhase(false);
        elevatorMotor.setInverted(false);

        // Sets up velocity and acceleration for motion magic
        elevatorMotor.configMotionCruiseVelocity(Constants.elevatorVel, Constants.timeout);
        elevatorMotor.configMotionAcceleration(Constants.elevatorAccel, Constants.timeout);

        // Sets current limits
        elevatorMotor.configContinuousCurrentLimit(12, timeout);
        elevatorMotor.configPeakCurrentLimit(15, timeout);
        elevatorMotor.configPeakCurrentDuration(100, timeout);
        elevatorMotor.enableCurrentLimit(true);

        // Zero encoders on fwd limit or bottom limit
        elevatorMotor.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 10);

        // Sets reverse soft limit
        // elevatorMotor.configReverseSoftLimitThreshold(RobotMap.elevatorMax,
        // RobotMap.timeout);
        elevatorMotor.configReverseSoftLimitEnable(false, Constants.timeout);

        // //Sets forward soft limit
        // elevatorMotor.configForwardSoftLimitThreshold(0, RobotMap.timeout);
        elevatorMotor.configForwardSoftLimitEnable(false, Constants.timeout);

    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new TeleopElevator());
    }

    public void percentOutput(double percent) {
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setHeight(double height) {
        elevatorMotor.configMotionCruiseVelocity(Constants.elevatorVel, Constants.timeout);
        elevatorMotor.configMotionAcceleration(Constants.elevatorAccel, Constants.timeout);
        elevatorMotor.set(ControlMode.MotionMagic, height);
    }

    public void controlHeight(double power, double initialPos) {
        elevatorMotor.configMotionCruiseVelocity(400, Constants.timeout);
        elevatorMotor.configMotionAcceleration(400, Constants.timeout);
        double targetPos = initialPos;
        double distanceToTravel = (int) (3 * 4096 / (Math.PI * 2)); // 3 inches max, 2 inch sprocket diameter
        targetPos += power * distanceToTravel;
        elevatorMotor.set(ControlMode.MotionMagic, targetPos);
    }

    public void groundHeight() {
        elevatorMotor.configMotionCruiseVelocity(Constants.elevatorDownVel, Constants.timeout);
        elevatorMotor.configMotionAcceleration(Constants.elevatorDownAccel, Constants.timeout);
        elevatorMotor.set(ControlMode.MotionMagic, 0);

    }

    public double getClosedLoopError() {
        return elevatorMotor.getClosedLoopError(0);
    }

    public void stop() {
        elevatorMotor.set(0);
    }

    public double getEncoderPos() {
        return elevatorMotor.getSensorCollection().getQuadraturePosition();
    }

    public double getEncoderVel() {
        return elevatorMotor.getSensorCollection().getQuadratureVelocity();
    }

    public double getMotorOutput() {
        return elevatorMotor.getMotorOutputPercent();
    }

    public void zeroEncoders() {
        elevatorMotor.setSelectedSensorPosition(0, pidIdx, timeout);
    }

    public boolean getForwardLimit() {
        return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimit() {
        return elevatorMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void zeroVelocity() {
        elevatorMotor.set(ControlMode.Velocity, 0);
    }
}
