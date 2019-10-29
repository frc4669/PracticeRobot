/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "ctre/Phoenix.h"
#include "frc/SpeedControllerGroup.h"
#include "frc/drive/MecanumDrive.h"
#include "RobotMap.h"

class DriveTrain : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

  WPI_TalonSRX leftFrontTalon{RobotMap::kLeftFrontMotor};
  WPI_TalonSRX leftBackTalon{RobotMap::kLeftBackMotor};
  WPI_TalonSRX rightFrontTalon{RobotMap::kRightFrontMotor};
  WPI_TalonSRX rightBackTalon{RobotMap::kRightBackMotor};
  
  frc::SpeedControllerGroup leftTalons{leftFrontTalon, leftBackTalon};
  frc::SpeedControllerGroup rightTalons{rightFrontTalon, rightBackTalon};

  frc::MecanumDrive mecanumDrive{leftFrontTalon, leftBackTalon, rightFrontTalon, rightBackTalon};


 public:
  DriveTrain();
  void InitDefaultCommand() override;
  void mecanumWheelDrive(double ySpeed, double xSpeed, double zRotation);
 
};
