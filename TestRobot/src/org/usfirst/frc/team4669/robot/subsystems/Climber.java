package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.ClimberCommand;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
 *
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private WPI_TalonSRX centerClimber = new WPI_TalonSRX(RobotMap.centerclimberID);
	private WPI_TalonSRX rightClimber = new WPI_TalonSRX(RobotMap.rightclimberID);
	private WPI_TalonSRX leftClimber = new WPI_TalonSRX(RobotMap.leftclimberID);
	public Accelerometer accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
	
	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ClimberCommand());
    }
    
    public void climbAll() {
    	// double 0 is placeholder for speed.
    	centerClimber.set(0);
    	rightClimber.set(0);
    	leftClimber.set(0);
    }
    
    // double 1 is placeholder for speed.
    public void climbCenter(Boolean climb) {
    	if (climb == true)
    	centerClimber.set(1);
    	else
    	centerClimber.set(0);
    }
    public void climbLeft(Boolean climb) {
    	if (climb == true)
    	leftClimber.set(1);
    	else
    	leftClimber.set(0);
    }
    public void climbRight(Boolean climb) {
    	if (climb == true)
    	rightClimber.set(1);
    	else
    	rightClimber.set(0);
    }
    
    public void climbBackwardsCenter(Boolean unwind) {
    	if (unwind == true)
    	centerClimber.set(-1);
    	else
    	centerClimber.set(0);
    }
    
    public void climbBackwardsLeft(Boolean unwind) {
    	if (unwind == true)
    	leftClimber.set(-1);
    	else
    	leftClimber.set(0);
    }
    public void climbBackwardsRight(Boolean unwind) {
    	if (unwind == true)
    	rightClimber.set(-1);
    	else
    	rightClimber.set(0);
    }
    
    public void climbForwardCenter(Boolean unwind) {
    	if (unwind == true)
    	centerClimber.set(1);
    	else
    	centerClimber.set(0);
    }
    
    public void climbForwardLeft(Boolean unwind) {
    	if (unwind == true)
    	leftClimber.set(1);
    	else
    	leftClimber.set(0);
    }
    public void climbForwardRight(Boolean unwind) {
    	if (unwind == true)
    	rightClimber.set(1);
    	else
    	rightClimber.set(0);
    }
   
    public double getaccelX() {
    	return accel.getX();
    }
    
    public double getaccelY() {
    	return accel.getY();
    }
    public double getaccelZ() {
    	return accel.getZ();
    }
    
    public void stop() {
    	centerClimber.set(0);
    	rightClimber.set(0);
    	leftClimber.set(0);
    }
    
    
    
}

