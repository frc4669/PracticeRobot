package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnTo extends Command {

    private double degree;

    public TurnTo(double degree) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrain);
        this.degree = degree;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        setTimeout(1.8);
        Robot.driveTrain.stop();
        Robot.driveTrain.enablePIDController(Robot.driveTrain.getGyroController());
        Robot.driveTrain.setTarget(Robot.driveTrain.getGyroController(), degree);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.driveTrain.tankDrive(Robot.driveTrain.getTurnOutput(), -Robot.driveTrain.getTurnOutput(), false);
        System.out.println(Robot.driveTrain.getPIDError(Robot.driveTrain.getGyroController()));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // Stop turning once robot turns past specified angle
        return Robot.driveTrain.getPIDDone(Robot.driveTrain.getGyroController()) || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.driveTrain.disablePIDController(Robot.driveTrain.getGyroController());
        Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

    public void setDegree(double degree) {
        this.degree = degree;
    }
}
