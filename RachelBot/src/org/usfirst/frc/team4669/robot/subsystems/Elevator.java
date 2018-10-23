package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.ElevatorGo;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class Elevator extends Subsystem {

	private WPI_TalonSRX elevator;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ElevatorGo());

    }


	public Elevator() {
    	super();
		elevator = new WPI_TalonSRX(RobotMap.elevator);
    	
    	elevator.configContinuousCurrentLimit(20, 0); //long-term current limit
    	elevator.configPeakCurrentLimit(22, 0);		  //short-term current limit
    	elevator.configPeakCurrentDuration(50, 0);    //how long is short-term?
    	elevator.enableCurrentLimit(true);
   
    }
    
    public void stop(){
    	elevator.set(0);
	}
    
    public void set(double value) {
    	elevator.set(ControlMode.PercentOutput, value);
    }
    
}

