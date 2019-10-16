/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <ctre::phoenix::motorcontrol::can::WPI_TalonSRX>

class DriveTrain : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  TalonSRX leftFrontTalon;
  TalonSRX leftBackTalon;
  TalonSRX rightFrontTalon;
  TalonSRX rightBackTalon;
  
  SpeedControllerGroup leftTalons;
  SpeedControllerGroup rightTalons;

  DifferentialDrive differentialDrive;

 public:
  DriveTrain();
  void InitDefaultCommand() override;
};
