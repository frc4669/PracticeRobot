package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.IntakeGo;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private WPI_TalonSRX rightIntake;
	private WPI_TalonSRX leftIntake;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new IntakeGo());

    }
    
    public Intake() {
    	super();
    	rightIntake = new WPI_TalonSRX(RobotMap.rightIntake);
    	leftIntake = new WPI_TalonSRX(RobotMap.leftIntake);
    	
    	rightIntake.setInverted(false);
	    leftIntake.setInverted(true);
    	
    	rightIntake.configContinuousCurrentLimit(20, 0);
    	rightIntake.configPeakCurrentLimit(22, 0);
    	rightIntake.configPeakCurrentDuration(50, 0);
    	rightIntake.enableCurrentLimit(true);
    	
    	leftIntake.configContinuousCurrentLimit(20, 0);
    	leftIntake.configPeakCurrentLimit(22, 0);
    	leftIntake.configPeakCurrentDuration(50, 0);
    	leftIntake.enableCurrentLimit(true);
    }
    
    public void stop(){
    	rightIntake.set(0);
    	leftIntake.set(0);
    	}
    
    public void set(double value) {
    	rightIntake.set(ControlMode.PercentOutput, value);
    	leftIntake.set(ControlMode.PercentOutput, value);
    }
}

