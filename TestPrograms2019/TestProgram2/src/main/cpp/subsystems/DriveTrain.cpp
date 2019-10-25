/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"
#include "RobotMap.h"
#include "commands/ArcadeDrive.h"

DriveTrain::DriveTrain() : Subsystem("ExampleSubsystem") {
  leftFrontTalon = new WPI_TalonSRX(RobotMap::kLeftFrontMotor);
  leftBackTalon = new WPI_TalonSRX(RobotMap::kLeftBackMotor);
  rightFrontTalon = new WPI_TalonSRX(RobotMap::kRightFrontMotor);
  rightBackTalon = new WPI_TalonSRX(RobotMap::kRightBackMotor);

  leftTalons = new frc::SpeedControllerGroup(*leftFrontTalon);
  rightTalons = new frc::SpeedControllerGroup(*rightFrontTalon);

  differentialDrive = new frc::DifferentialDrive(*leftTalons, *rightTalons);
}

void DriveTrain::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  SetDefaultCommand(new ArcadeDrive());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void DriveTrain::arcadeDrive(double moveSpeed, double rotateSpeed)
{
  differentialDrive->ArcadeDrive(moveSpeed, rotateSpeed);
}
