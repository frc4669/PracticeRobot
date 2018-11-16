package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.Constants;
import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorCommand extends Command {

    public ElevatorCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.controlElevatorMotor(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    
        if (Robot.oi.getLeftRawButton(3))
    		Robot.elevator.setHeight(0);
        else if (Robot.oi.getLeftRawButton(2))
        	Robot.elevator.setHeight(Constants.elevatorHeightMiddle);
        else
        	Robot.elevator.controlElevatorMotor(Robot.oi.leftX());
        
        
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
