package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeCommand extends Command {

    public IntakeCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.controlMotors(0);
    }

   // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.f310.getButton(F310.redButton))
    		Robot.intake.controlMotors(0.4);    		
    	else if (Robot.f310.getButton(F310.orangeButton))
    		Robot.intake.controlMotors(-0.4);
    	else
    		Robot.intake.controlMotors(0);
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
