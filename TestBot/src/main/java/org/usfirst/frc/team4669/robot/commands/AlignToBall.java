/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.F310;
import org.usfirst.frc.team4669.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.networktables.NetworkTableEntry;

public class AlignToBall extends Command {

  int centerX = 160;
  int centerY = 100;

  public AlignToBall() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.stop();
    Robot.driveTrain.enablePIDController(Robot.driveTrain.getVisionTurnController());
    Robot.driveTrain.setTarget(Robot.driveTrain.getVisionTurnController(), centerX);
    Robot.driveTrain.enablePIDController(Robot.driveTrain.getVisionDistanceController());
    Robot.driveTrain.setTarget(Robot.driveTrain.getVisionDistanceController(), 200);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // // // turn toward the ball and try to get it to stop
    // if (Math.abs(x - centerX) <= 10) { // when the ball is within range ofcenterX
    // (150~170)
    // Robot.driveTrain.stop(); // stops robot
    // Robot.driveTrain.arcadeDrive(0.2, 0, false);
    // if (width > 200) {
    // Robot.driveTrain.stop();
    // }
    // } else if (x <= 150) {
    // Robot.driveTrain.arcadeDrive(0, -0.4, false); // turns robot if not within
    // range
    // } else if (x >= 170) {
    // Robot.driveTrain.arcadeDrive(0, 0.4, false);
    // }
    if (Robot.driveTrain.getPIDDone(Robot.driveTrain.getVisionTurnController())) {
      Robot.driveTrain.arcadeDrive(Robot.driveTrain.getVisionDistanceOutput(), 0, false);
    } else {
      Robot.driveTrain.arcadeDrive(0, -Robot.driveTrain.getVisionTurnOutput(), false);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.f310.getButton(F310.redButton) || !Robot.driveTrain.isObjectDetected()
        || Robot.driveTrain.getPIDDone(Robot.driveTrain.getVisionTurnController())
            && Robot.driveTrain.getPIDDone(Robot.driveTrain.getVisionDistanceController()));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.disablePIDController(Robot.driveTrain.getVisionDistanceController());
    Robot.driveTrain.disablePIDController(Robot.driveTrain.getVisionTurnController());
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
