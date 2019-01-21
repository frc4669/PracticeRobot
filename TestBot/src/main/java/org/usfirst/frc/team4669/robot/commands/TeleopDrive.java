package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleopDrive extends Command {

    boolean turnRunning = false;
    boolean angleSet = false;
    double angle = 0.0;

    public TeleopDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.driveTrain.stop();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        if (Robot.f310.getDPadPOV() != -1 && !turnRunning) {
            System.out.println("Setting turn angle");
            double turnAngle = Robot.f310.getDPadPOV();
            Robot.driveTrain.enablePIDController(Robot.driveTrain.getGyroController());
            Robot.driveTrain.setTarget(Robot.driveTrain.getGyroController(), turnAngle);
            turnRunning = true;
        } else if (turnRunning && Robot.driveTrain.getPIDDone(Robot.driveTrain.getGyroController())) {
            System.out.println("Stopping turning");
            Robot.driveTrain.disablePIDController(Robot.driveTrain.getGyroController());
            Robot.driveTrain.stop();
            turnRunning = false;
        } else if (turnRunning) {
            Robot.driveTrain.tankDrive(Robot.driveTrain.getTurnOutput(), -Robot.driveTrain.getTurnOutput(), false);
        } else if (Robot.f310.getButton(F310.leftShoulderButton) && Robot.f310.getDPadPOV() == -1 && !turnRunning) {
            if (!angleSet) {
                System.out.println("Setting straight angle");
                angle = Robot.driveTrain.getAngle();
                angleSet = true;
            }
            Robot.driveTrain.driveStraightGyro(Robot.f310.getLeftY(), angle, Constants.kPStraightGyro);
        } else {
            Robot.driveTrain.arcadeDrive(Robot.f310.getLeftY(), 0.6 * Robot.f310.getRightX(), false);
            angleSet = false;
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.driveTrain.stop();
    }
}
