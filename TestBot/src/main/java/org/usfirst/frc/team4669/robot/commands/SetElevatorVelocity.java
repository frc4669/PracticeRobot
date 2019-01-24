/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SetElevatorVelocity extends Command {
  double velocityL = 0;
  double velocityR = 0;

  public SetElevatorVelocity(double velocityL, double velocityR) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.velocityL = velocityL;
    this.velocityR = velocityR;
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevator.setVelocity(Robot.elevator.getLeftMotor(), velocityL);
    Robot.elevator.setVelocity(Robot.elevator.getRightMotor(), velocityR);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.getLeftRawButton(10);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
