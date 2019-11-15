/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"
#include "RobotMap.h"
#include "commands/MecanumDrive.h"
#include "Robot.h"

DriveTrain::DriveTrain() : Subsystem("ExampleSubsystem") {
  gyro = new frc::ADXRS450_Gyro();
}

void DriveTrain::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  //leftFrontTalon.SetStatusFramePeriod(ctre::phoenix::motorcontrol::Status_3_Quadrature,10/*update period in ms*/);
  //rightFrontTalon.SetStatusFramePeriod(ctre::phoenix::motorcontrol::Status_3_Quadrature,10/*update period in ms*/);
  SetDefaultCommand(new MecanumDrive());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

double changeSpeedFast(double speed){
  double changedSpeed = 0.8*pow(speed,2);
  if (speed > 0) {
    return changedSpeed;
  } 
  else {
    return -1*changedSpeed;
  }
}

double changeSpeedSlow(double speed){
  double changedSpeed = 0.5*pow(speed,2);
  if (speed > 0) {
    return changedSpeed;
  } 
  else {
    return -1*changedSpeed;
  }
}

double DriveTrain::getGyroAngle()
{
  return gyro->GetAngle();
}

void DriveTrain::DifferentialWheelDrive(double ySpeed, double xSpeed, double zRotation, double gyroAngle){
  DifferentialDrive.ArcadeDrive(xSpeed, zRotation);
}


// void DriveTrain::mecanumWheelDrive(double ySpeed, double xSpeed, double zRotation, double gyroAngle)
// {
//   if (Robot::f310.getButton(Robot::f310.right_shoulder_button))
//   {
//     mecanumDrive.DriveCartesian(changeSpeedSlow(ySpeed), changeSpeedSlow(xSpeed), changeSpeedSlow(zRotation), gyroAngle);
//   }
//   else 
//   {
//     mecanumDrive.DriveCartesian(changeSpeedFast(ySpeed), changeSpeedFast(xSpeed), changeSpeedFast(zRotation), gyroAngle);
//   }
// }

// void DriveTrain::mecanumWheelDrive(double ySpeed, double xSpeed, double zRotation)
// {
//   if (Robot::f310.getButton(Robot::f310.right_shoulder_button))
//   {
//     mecanumDrive.DriveCartesian(changeSpeedSlow(ySpeed), changeSpeedSlow(xSpeed), changeSpeedSlow(zRotation));
//   }
//   else 
//   {
//     mecanumDrive.DriveCartesian(changeSpeedFast(ySpeed), changeSpeedFast(xSpeed), changeSpeedFast(zRotation));
//   }
// }

// void Drivetrain::driveMotionMagic(double targetEncPosition) 
// {
//     leftFrontTalon.set(ControlMode.MotionMagic, targetEncPosition);
//     rightFrontTalon.set(ControlMode.MotionMagic, targetEncPosition);
//     leftBackTalon.set(ControlMode.MotionMagic, targetEncPosition);
//     rightBackTalon.set(ControlMode.MotionMagic, targetEncPosition);
// }
