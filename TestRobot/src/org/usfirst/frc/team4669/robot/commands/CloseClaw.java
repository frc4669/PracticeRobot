//package org.usfirst.frc.team4669.robot.commands;
//import edu.wpi.first.wpilibj.command.Command;
//import org.usfirst.frc.team4669.robot.Robot;
//
//public class CloseClaw extends Command {
//
//	public CloseClaw() {
//        // Use requires() here to declare subsystem dependencies
//        // eg. requires(chassis);
//		requires(Robot.grabber);
//		setTimeout(.9);
//    }
//
//    // Called just before this Command runs the first time
//    protected void initialize() {
//    	Robot.grabber.close();
//    }
//
//    // Called repeatedly when this Command is scheduled to run
//    protected void execute() {
//    }
//
//    // Make this return true when this Command no longer needs to run execute()
//    protected boolean isFinished() {
//        return isTimedOut();
//    }
//
//    // Called once after isFinished returns true
//    protected void end() {
//    	Robot.grabber.stop();
//    }
//
//    // Called when another command which requires one or more of the same
//    // subsystems is scheduled to run
//    protected void interrupted() {
//    	end();
//    }
//}
//
