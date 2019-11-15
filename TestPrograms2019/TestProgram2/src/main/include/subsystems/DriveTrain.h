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
#include "frc/drive/DifferentialDrive.h"
#include "RobotMap.h"
#include "frc/ADXRS450_Gyro.h"

class DriveTrain : public frc::Subsystem {
//  private:
  public:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

  frc::ADXRS450_Gyro * gyro;

  WPI_TalonSRX leftFrontTalon{RobotMap::kLeftFrontMotor};
  //WPI_TalonSRX leftBackTalon{RobotMap::kLeftBackMotor};
  WPI_TalonSRX rightFrontTalon{RobotMap::kRightFrontMotor};
  //WPI_TalonSRX rightBackTalon{RobotMap::kRightBackMotor};
  
  frc::SpeedControllerGroup leftTalons{leftFrontTalon/*, leftBackTalon*/};
  frc::SpeedControllerGroup rightTalons{rightFrontTalon/*, rightBackTalon*/};

  // frc::MecanumDrive mecanumDrive{leftFrontTalon, /*leftBackTalon,*/ rightFrontTalon/*, rightBackTalon*/};
  frc::DifferentialDrive DifferentialDrive{leftFrontTalon, /*leftBackTalon,*/ rightFrontTalon/*, rightBackTalon*/};


 public:
  DriveTrain();
  void InitDefaultCommand() override;
  void DifferentialWheelDrive(double ySpeed, double xSpeed, double zRotation, double gyroAngle);
  void driveMotionMagic(double targetEncoderPosition);

  double getGyroAngle();

  
 
};
