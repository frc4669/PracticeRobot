/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MecanumDrive.h"
#include "Robot.h"

MecanumDrive::MecanumDrive() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_drivetrain);
}

// Called just before this Command runs the first time
void MecanumDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MecanumDrive::Execute() {
  double ySpeed = Robot::f310.getLeftX();
  double xSpeed = -Robot::f310.getLeftY();
  double zRotation = Robot::f310.getRightX();

  Robot::m_drivetrain.mecanumWheelDrive(ySpeed, xSpeed, zRotation);
}


// Make this return true when this Command no longer needs to run execute()
bool MecanumDrive::IsFinished() { 
  return false; 
}

// Called once after isFinished returns true
void MecanumDrive::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MecanumDrive::Interrupted() {}
