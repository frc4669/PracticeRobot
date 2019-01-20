package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimberCommand extends Command {

    public ClimberCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.climber);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// robotmap.stopClimb is placeholder for which button on controller to be pressed to stop climb
       	if (Robot.oi.getLeftRawButton(RobotMap.stopClimb)) {
    		Robot.climber.stop();
    	} else {
    		// RobotMap.climberButton is a placeholder for which button to press for climb functions
	    	if (Robot.oi.getLeftRawButton(RobotMap.climberButton)) {
	    		// System.out.println(Robot.climber.getaccelX()+" "+Robot.climber.getaccelY());
	    		if (Math.abs(Robot.climber.getaccelX())<0.1&&Math.abs(Robot.climber.getaccelY())<0.1){
	    			Robot.climber.climbAll();
	    		} else if(Robot.climber.getaccelY()<-0.1){
	    			Robot.climber.climbCenter(false);
	    			Robot.climber.climbLeft(true);
	    			Robot.climber.climbRight(true);
	    		} else if(Robot.climber.getaccelY()>0.1){
	    			Robot.climber.climbLeft(false);
	    			Robot.climber.climbRight(false);
	    			Robot.climber.climbCenter(true);
	    		} else if(Robot.climber.getaccelX()>0.1){
	    			Robot.climber.climbCenter(false);
	    			Robot.climber.climbRight(false);
	    			Robot.climber.climbLeft(true);
	    		} else if(Robot.climber.getaccelX()<-0.1){
	    			Robot.climber.climbCenter(false);
	    			Robot.climber.climbLeft(false);
	    			Robot.climber.climbRight(true);
	    		}
	    	} else {
				Robot.climber.stop();
	    	}
    	}
    }
    

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
