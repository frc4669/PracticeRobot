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

class DriveTrain : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

  WPI_TalonSRX * leftFrontTalon;
  WPI_TalonSRX * leftBackTalon;
  WPI_TalonSRX * rightFrontTalon;
  WPI_TalonSRX * rightBackTalon;
  
  frc::SpeedControllerGroup * leftTalons;
  frc::SpeedControllerGroup * rightTalons;

  frc::MecanumDrive * mecanumDrive;

 public:
  DriveTrain();
  void InitDefaultCommand() override;
  void mecanumWheelDrive(double ySpeed, double xSpeed, double zRotation);
  ~DriveTrain();
};
