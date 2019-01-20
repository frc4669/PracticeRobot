package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.Constants;
import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MotionMagicTest extends Command {
	
	double distance;

    public MotionMagicTest() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	distance = 60; // Make robot move a distance in inches
    	requires(Robot.driveTrain);
    }
    
    public MotionMagicTest(int distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.distance = distance;
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Starting motion magic command");
    	Robot.driveTrain.stop();
    	//Sets encoder position to 0
    	System.out.println("Sets encoder position to 0");
    	Robot.driveTrain.setLeftSelectedSensorPosition(0); 
    	Robot.driveTrain.setRightSelectedSensorPosition(0);
    	//Drive robot to the set position
    	System.out.println("Running motion magic");
    	distance *= Constants.inchToEncoderUnits;
    	System.out.println("Target position in encoder units: " + distance);
    	Robot.driveTrain.driveMotionMagic(distance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.driveTrain.motionMagicIsDone(distance);
//    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
