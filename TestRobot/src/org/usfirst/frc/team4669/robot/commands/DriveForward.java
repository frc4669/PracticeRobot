//package org.usfirst.frc.team4669.robot.commands;
//
//import org.usfirst.frc.team4669.robot.Robot;
//import org.usfirst.frc.team4669.robot.subsystems.DriveTrain;
//
//import edu.wpi.first.wpilibj.command.TimedCommand;
//
///**
// *
// */
//public class DriveForward extends TimedCommand {
//
//	private DriveTrain driveTrain;
//	
//    public DriveForward(double timeout) {
//        super(timeout);
//        // Use requires() here to declare subsystem dependencies
//        // eg. requires(chassis);
//        driveTrain = Robot.driveTrain;
//    	requires(driveTrain);
//    }
//
//    // Called just before this Command runs the first time
//    protected void initialize() {
//    }
//
//    // Called repeatedly when this Command is scheduled to run
//    protected void execute() {
//    	driveTrain.driveForward(-0.5, -0.5);
//    }
//
//    // Called once after timeout
//    protected void end() {
//    	driveTrain.driveForward(0,0);
//    }
//
//    // Called when another command which requires one or more of the same
//    // subsystems is scheduled to run
//    protected void interrupted() {
//    }
//}