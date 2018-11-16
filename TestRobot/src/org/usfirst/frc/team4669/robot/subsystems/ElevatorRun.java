package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.Constants;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.ElevatorCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ElevatorRun extends Subsystem {
	private WPI_TalonSRX elevatorMotor;
	
	public ElevatorRun() {
		elevatorMotor = new WPI_TalonSRX(RobotMap.elevatorMotor);
		elevatorMotor.setNeutralMode(NeutralMode.Brake);
		elevatorMotor.setInverted(false);
		
		//set sensor and configurations for targeted measurements:
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		elevatorMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		elevatorMotor.config_kF(Constants.kSlotIdx, Constants.kF, Constants.kTimeoutMs);
		elevatorMotor.config_kP(Constants.kSlotIdx, Constants.kP, Constants.kTimeoutMs);
		elevatorMotor.config_kI(Constants.kSlotIdx, Constants.kI, Constants.kTimeoutMs);
		elevatorMotor.config_kD(Constants.kSlotIdx, Constants.kD, Constants.kTimeoutMs); 
		elevatorMotor.configMotionCruiseVelocity(Constants.elevatorVel, Constants.kTimeoutMs);
		elevatorMotor.configMotionAcceleration(Constants.elevatorAcc, Constants.kTimeoutMs);
		
		elevatorMotor.configNominalOutputForward(0, RobotMap.timeout);
		elevatorMotor.configNominalOutputReverse(0, RobotMap.timeout);
		elevatorMotor.configPeakOutputForward(1, RobotMap.timeout);
		elevatorMotor.configPeakOutputReverse(-1, RobotMap.timeout);
		elevatorMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
	

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ElevatorCommand());
    }
    
    public void controlElevatorMotor(double percentage) {
    	elevatorMotor.set(ControlMode.PercentOutput, percentage); 
    }
    
    //make method that sets motors to Motion magic mode and moves to a set location
    public void setHeight(int height){
    	elevatorMotor.set(ControlMode.MotionMagic, height); 
    }
    
    public void setMotionVelAccel(int velocity,int accel) {
		elevatorMotor.configMotionCruiseVelocity(velocity,Constants.kTimeoutMs);
		elevatorMotor.configMotionAcceleration(accel,Constants.kTimeoutMs);
    }

}

