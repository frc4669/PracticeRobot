package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.F310;

/**
 *
 */
public class GearShiftCommand extends Command {

    public GearShiftCommand() {
        // Use requires() here to declare subsystem dependencies
    	// eg. requires(chassis);
        requires(Robot.gearShifter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.f310.getButton(F310.redButton)){
            if(Robot.gearShifter.getHighGear()){
                Robot.gearShifter.lowGear();
            }
            Robot.gearShifter.highGear();
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