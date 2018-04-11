package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private WPI_TalonSRX topRight;
    private WPI_TalonSRX botRight;
    private WPI_TalonSRX topLeft;
    private WPI_TalonSRX botLeft;
    
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public DriveTrain() {
    	super();
    	topRight = new WPI_TalonSRX(RobotMap.topRight);
    	botRight = new WPI_TalonSRX(RobotMap.botRight);
    	topLeft = new WPI_TalonSRX(RobotMap.topLeft);
    	botLeft = new WPI_TalonSRX(RobotMap.botLeft);
    	//putting stuff in variable
    	
    	topRight.configContinuousCurrentLimit(20, 0); //long-term current limit
    	topRight.configPeakCurrentLimit(22, 0);		  //short-term current limit
    	topRight.configPeakCurrentDuration(50, 0);   //how long is short-term?
    	topRight.enableCurrentLimit(true);
    	
    	botRight.configContinuousCurrentLimit(20, 0);
    	botRight.configPeakCurrentLimit(22, 0);
    	botRight.configPeakCurrentDuration(50, 0);
    	botRight.enableCurrentLimit(true);
    	
    	topLeft.configContinuousCurrentLimit(20, 0);
    	topLeft.configPeakCurrentLimit(22, 0);
    	topLeft.configPeakCurrentDuration(50, 0);
    	topLeft.enableCurrentLimit(true);
    	
    	botLeft.configContinuousCurrentLimit(20, 0);
    	botLeft.configPeakCurrentLimit(22, 0);
    	botLeft.configPeakCurrentDuration(50, 0);
    	botLeft.enableCurrentLimit(true);
    	
    	botRight.set(ControlMode.Follower, topRight.getDeviceID());
		botLeft.set(ControlMode.Follower, topLeft.getDeviceID());
    }
    
    public void stop(){
    	topRight.set(0);
    	topLeft.set(0);
    }
}

