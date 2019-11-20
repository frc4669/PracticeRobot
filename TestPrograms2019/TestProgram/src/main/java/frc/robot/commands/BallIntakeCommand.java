package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.F310;

/**
 *
 */
public class BallIntakeCommand extends Command {

    public BallIntakeCommand() {
        // Use requires() here to declare subsystem dependencies
    	// eg. requires(chassis);
        requires(Robot.ballIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.f310.getButton(F310.greenButton)){
    		//Attempt at intake
    		Robot.ballIntake.intake();
    	} 
		else if(Robot.f310.getButton(F310.orangeButton)){
    		Robot.ballIntake.releaseBall();
    	}
		else {
    		Robot.ballIntake.stopIntake();
        }
        
        if(Robot.f310.getButton(F310.leftShoulderButton)){
    		//Attempt at open/close jaw
    		Robot.ballIntake.openJaw();
    	} 
		else if(Robot.f310.getButton(F310.rightShoulderButton)){
    		Robot.ballIntake.closeJaw();
    	}
		else {
    		Robot.ballIntake.stopJaw();
        }

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.ballIntake.stopIntake();
    }
    
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}