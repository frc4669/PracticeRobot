/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MecanumDrive.h"
#include "Robot.h"
#include <iostream>
#include "Constants.h"

MecanumDrive::MecanumDrive() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_drivetrain);
}

// Called just before this Command runs the first time
void MecanumDrive::Initialize() {}

int right_last_encoder_value;
int left_last_encoder_value;

bool is_first_iteration = false;

bool autonomous_mode_engaged = false;
float autonomous_distance_target = 24;
float autonomous_distance_driven = 0;


// Called repeatedly when this Command is scheduled to run
void MecanumDrive::Execute() {
  double ySpeed = Robot::f310.getLeftX();
  double xSpeed = -Robot::f310.getLeftY();
  double zRotation = Robot::f310.getRightX();
  double gyroAngle = Robot::m_drivetrain.getGyroAngle();
  bool autonomous_mode_triggered = Robot::f310.getButtonReleased(Robot::f310.red_button);

  auto left_sensor_collection = Robot::m_drivetrain.leftFrontTalon.GetSensorCollection();
  int left_raw_encoder_value = left_sensor_collection.GetQuadraturePosition();
  float left_travel_distance = 0;

  auto right_sensor_collection = Robot::m_drivetrain.rightFrontTalon.GetSensorCollection();
  //the right moter is spining backwards so we need to multiply the encoder value by -1
  int right_raw_encoder_value = right_sensor_collection.GetQuadraturePosition() * -1;
  float right_travel_distance = 0;

  if (is_first_iteration) {
    left_last_encoder_value = left_raw_encoder_value;
    right_last_encoder_value = right_raw_encoder_value;
  }
  else {
    // this is not the first iteration, so we have valid last values for left and right encoders
    int difference_in_left_encoder_value = left_raw_encoder_value - left_last_encoder_value;
    left_travel_distance = difference_in_left_encoder_value *  Constants::inchesOverEncoderTicks;
    left_last_encoder_value = left_raw_encoder_value;

    int difference_in_right_encoder_value = right_raw_encoder_value - right_last_encoder_value;
    right_travel_distance = difference_in_right_encoder_value *  Constants::inchesOverEncoderTicks;
    right_last_encoder_value = right_raw_encoder_value;
  }
 
  if (autonomous_mode_engaged) {
    //drive forward X inches
    autonomous_distance_driven += abs(right_travel_distance + left_travel_distance) / 2;
    //disable autonomous mode
    if (autonomous_distance_driven >= autonomous_distance_target) {
      autonomous_mode_engaged = false;
    }
    else {
      Robot::m_drivetrain.DifferentialWheelDrive(0, -0.45, 0,0);
    }
  }
  else {
    //Add gyroAngle parameter to drive relative to field
    Robot::m_drivetrain.DifferentialWheelDrive(0.5 * ySpeed, 0.5 * xSpeed, zRotation, 0);
    autonomous_distance_driven = 0;
    if (autonomous_mode_triggered) {
      autonomous_mode_engaged = true;
    }
  }

  if (Robot::f310.getButton(Robot::f310.orange_button))
  {
    Robot::m_drivetrain.leftFrontTalon.SetSelectedSensorPosition(0);
    Robot::m_drivetrain.rightFrontTalon.SetSelectedSensorPosition(0);
  }

  //print current speed of each wheel here
  float left_right_delta =  left_travel_distance - right_travel_distance;
  //std::cout << "Left travel distance is " << left_travel_distance << " inches" << std::endl;
  //std::cout << "Right travel distance is " << right_travel_distance << " inches" << std::endl;
  //std::cout << "Difference between Left and Right: " << left_right_delta << "("<< left_right_delta * 100 / std::max(left_travel_distance,right_travel_distance) << "%)"<< std::endl;
  // std::cout << "Right back wheel encoder value is " << Robot::m_drivetrain.rightBackTalon.GetSensorCollection().GetQuadraturePosition() << std::endl;
  std::cout << "engaged: " << autonomous_mode_engaged << " trigger: " <<
  autonomous_mode_triggered << " distance_driven: " << autonomous_distance_driven << std::endl
  << "left encoder value: " << left_raw_encoder_value << std::endl
  << "right encoder value: " << right_raw_encoder_value << std::endl;


  // thing.GetEncoder()
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
