/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.ArmCommand;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX shoulderMotor;
  private WPI_TalonSRX elbowMotor;
  private WPI_TalonSRX wristMotor;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ArmCommand());
  }

  public Arm() {
    shoulderMotor = new WPI_TalonSRX(RobotMap.shoulderMotor);
    elbowMotor = new WPI_TalonSRX(RobotMap.elbowMotor);
    wristMotor = new WPI_TalonSRX(RobotMap.wristMotor);
    setCurrentLimit(elbowMotor);
    setCurrentLimit(wristMotor);
    setCurrentLimit(shoulderMotor);

    setMotorOutputs(shoulderMotor, 0, 0.8);
    setMotorOutputs(elbowMotor, 0, 0.8);
    setMotorOutputs(wristMotor, 0, 0.8);

    shoulderMotor.setInverted(true);
    elbowMotor.setInverted(false);
    wristMotor.setInverted(false);

    shoulderMotor.configOpenloopRamp(3.2);
    elbowMotor.configOpenloopRamp(1.5);

  }

  /**
   * Method to control the arm motors individually
   * 
   * @param shoulderMotorPower Controls the shoulder motor, inputs [-1,1]
   * @param elbowMotorPower    Controls the elbow motor, inputs [-1,1]
   * @param wristMotorPower    Controls the wrist motor, inputs [-1,1]
   */
  public void motorControl(double shoulderMotorPower, double elbowMotorPower, double wristMotorPower) {
    shoulderMotor.set(shoulderMotorPower);
    elbowMotor.set(elbowMotorPower);
    wristMotor.set(wristMotorPower);
  }

  public void stop() {
    motorControl(0.0, 0.0, 0.0);
  }

  public void setCurrentLimit(TalonSRX talon) {
    talon.configContinuousCurrentLimit(Constants.continuousCurrentLimitArm, Constants.timeout);
    talon.configPeakCurrentLimit(Constants.peakCurrentLimitArm, Constants.timeout);
    talon.configPeakCurrentDuration(Constants.currentDuration, Constants.timeout);
    talon.enableCurrentLimit(true);
  }

  public void setMotorOutputs(TalonSRX talon, double nominalOutput, double peakOutput) {
    talon.configNominalOutputForward(nominalOutput);
    talon.configNominalOutputReverse(-nominalOutput);
    talon.configPeakOutputForward(peakOutput);
    talon.configPeakOutputReverse(-peakOutput);
  }

  // public double getAngleShoulder(double x, double y) {
  // double a1 = Constants.shoulderLength;
  // double a2 = Constants.elbowLength;
  // double a3 = Constants.wristLength;

  // double x1 = x - a3; // Target length minus wrist length

  // double angleRad = Math.acos((Math.pow(a2, 2) - Math.pow(a1, 2) - Math.pow(x1,
  // 2) - Math.pow(y, 2))
  // / (-2 * a1 * Math.sqrt(Math.pow(x1, 2) + Math.pow(y, 2))));

  // double angleDeg = Math.toDegrees(angleRad);
  // return angleDeg;
  // }

  // public double getAngleElbow(double x, double y) {
  // double a1 = Constants.shoulderLength;
  // double a2 = Constants.elbowLength;
  // double a3 = Constants.wristLength;

  // double x1 = x - a3; // Target length minus wrist length

  // double angleRad = 180
  // - Math.acos((Math.pow(x1, 2) + Math.pow(y, 2) - Math.pow(a1, 2) -
  // Math.pow(a2, 2)) / (-1 * a1 * a2));

  // double angleDeg = Math.toDegrees(angleRad);
  // return angleDeg;
  // }

}
