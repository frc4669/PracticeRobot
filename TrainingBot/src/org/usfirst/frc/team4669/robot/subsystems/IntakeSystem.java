package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.IntakeCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class IntakeSystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

   	private WPI_TalonSRX rightMotor;
	private WPI_TalonSRX leftMotor;
	
	public IntakeSystem() {
    	rightMotor = new WPI_TalonSRX(RobotMap.intakeRightMotor);
		rightMotor.setNeutralMode(NeutralMode.Brake);
		leftMotor = new WPI_TalonSRX(RobotMap.intakeLeftMotor);
		leftMotor.setNeutralMode(NeutralMode.Brake);
		leftMotor.setInverted(true);
		rightMotor.setInverted(false);
    }
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new IntakeCommand());
    }
    
    public void controlMotors(double percentage) {
    	leftMotor.set(ControlMode.PercentOutput, percentage);
		rightMotor.set(ControlMode.PercentOutput, percentage);
    }
    
}

