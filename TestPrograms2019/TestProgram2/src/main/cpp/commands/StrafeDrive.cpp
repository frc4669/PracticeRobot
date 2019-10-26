/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/StrafeDrive.h"
#include "Robot.h"

StrafeDrive::StrafeDrive() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::m_drivetrain);
}

// Called just before this Command runs the first time
void StrafeDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void StrafeDrive::Execute() {
  double ySpeed = 1;
  double xSpeed = 0;
  double zRotation = 0;
  
  if (Robot::f310->getButton(Robot::f310->red_button))
  {
    Robot::m_drivetrain->strafeDrive(ySpeed, xSpeed, zRotation);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool StrafeDrive::IsFinished() { 
  return false; 
}

// Called once after isFinished returns true
void StrafeDrive::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void StrafeDrive::Interrupted() {}
