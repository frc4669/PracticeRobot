/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"
#include "include/RobotMap.h"


DriveTrain::DriveTrain() : Subsystem("ExampleSubsystem") {
  leftFrontTalon(RobotMap::kLeftFrontTalon);
  leftBackTalon(RobotMap::kLeftBackTalon);
  rightFrontTalon(RobotMap::kRightFrontTalon);
  rightBackTalon(RobotMap::kRightBackTalon);

  leftTalons(leftFrontTalon, leftBackTalon);
  rightTalons(rightFrontTalon, rightFrontTalon);

}

void DriveTrain::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
