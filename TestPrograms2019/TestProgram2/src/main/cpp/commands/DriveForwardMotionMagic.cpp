/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveForwardMotionMagic.h"
#include "Robot.h"
#include "Constants.h"

DriveForwardMotionMagic::DriveForwardMotionMagic(double distance) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_drivetrain);
  dDistance = distance;
}

// Called just before this Command runs the first time
void DriveForwardMotionMagic::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveForwardMotionMagic::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool DriveForwardMotionMagic::IsFinished() { 
  return ((abs(dDistance * Constants::encoderTicksOverInches - Robot::m_drivetrain.rightFrontTalon.GetSensorCollection().GetQuadraturePosition()) < Constants::driveTolerance 
    && abs(dDistance * Constants::encoderTicksOverInches - Robot::m_drivetrain.leftFrontTalon.GetSensorCollection().GetQuadraturePosition()) < Constants::driveTolerance) || Robot::f310.getButton(Robot::f310.red_button));
  // return ((Math.abs(distance * Constants.inchToEncoderDrive - Robot.driveTrain.getFrontRightEncoder()) < Constants.driveTolerance 
  //   && Math.abs(distance * Constants.inchToEncoderDrive - Robot.driveTrain.getFrontLeftEncoder()) < Constants.driveTolerance) || Robot.f310.getButton(F310.redButton));

  return true;
}

// Called once after isFinished returns true
void DriveForwardMotionMagic::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveForwardMotionMagic::Interrupted() {}
