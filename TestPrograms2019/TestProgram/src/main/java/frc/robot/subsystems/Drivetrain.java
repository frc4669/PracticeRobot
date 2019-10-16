/*
------------------------------------------------------------------------------
 Copyright (c) 2018 FIRST. All Rights Reserved.                             
Open Source Software - may be modified and shared by FRC teams. The code   
must be accompanied by the FIRST BSD license file in the root directory of 
the project.                                                               
----------------------------------------------------------------------------
*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import frc.robot.F310;
import frc.robot.Robot;
import frc.robot.commands.DriveArcade;

/**
* Add your docs here.
*/
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX leftFrontTalon = null;
  WPI_TalonSRX leftBackTalon = null;
  WPI_TalonSRX rightFrontTalon = null;
  WPI_TalonSRX rightBackTalon = null;

  DifferentialDrive differentialDrive = null;
  DifferentialDrive differentialDriveStrafe = null;
  
  public Drivetrain() {
    leftFrontTalon = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT_TALON);
    leftBackTalon = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_BACK_TALON);
    rightFrontTalon = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT_TALON);
    rightBackTalon = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_BACK_TALON);

    //Motor controller groups for normal forward and backward driving
    SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftFrontTalon, leftBackTalon);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightFrontTalon, rightBackTalon);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  }

  public double changeSpeedFast(double speed){
    double changedSpeed = 0.8*Math.pow(speed,2);
    if(speed > 0) {
      return changedSpeed;
    } 
    else {
      return -1*changedSpeed;
    }
  }

  public double changeSpeedSlow(double speed){
    double changedSpeed = 0.5*Math.pow(speed,2);
    if(speed > 0) {
      return changedSpeed;
    } 
    else {
      return -1*changedSpeed;
    }
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    //boolean rightShoulderReleased = Robot.f310.getButtonReleased(F310.rightShoulderButton);

    if (Robot.f310.getButton(F310.rightShoulderButton)) 
    {
      differentialDrive.arcadeDrive(changeSpeedSlow(moveSpeed), changeSpeedSlow(rotateSpeed));  
    }
    else 
    {
      differentialDrive.arcadeDrive(changeSpeedFast(moveSpeed), changeSpeedFast(rotateSpeed)); 
    }
    
  }

  public void strafeDrive(double moveSpeed, double rotateSpeed) {
    
    if (Robot.f310.getButton(F310.rightShoulderButton)) 
    {
      differentialDriveStrafe.arcadeDrive(changeSpeedSlow(moveSpeed), changeSpeedSlow(rotateSpeed));  
    }
    else 
    {
      differentialDriveStrafe.arcadeDrive(changeSpeedFast(moveSpeed), changeSpeedFast(rotateSpeed)); 
    }
    
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveArcade());
   }
} 
