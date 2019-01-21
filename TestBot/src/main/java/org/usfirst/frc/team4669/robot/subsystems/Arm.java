/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

    talonConfig(shoulderMotor, false);
    talonConfig(elbowMotor, false);
    talonConfig(wristMotor, false);

    setCurrentLimit(elbowMotor);
    setCurrentLimit(wristMotor);
    setCurrentLimit(shoulderMotor);

    setMotorOutputs(shoulderMotor, 0, 0.8);
    setMotorOutputs(elbowMotor, 0, 0.8);
    setMotorOutputs(wristMotor, 0, 1);

    setMotionVelAccel(shoulderMotor, Constants.shoulderVel, Constants.shoulderAccel);
    setMotionVelAccel(elbowMotor, Constants.elbowVel, Constants.elbowAccel);
    setMotionVelAccel(wristMotor, Constants.wristVel, Constants.wristAccel);

    zeroEncoders();

    shoulderMotor.configOpenloopRamp(2);
    elbowMotor.configOpenloopRamp(0.25);

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

  /**
   * Method to get the angle to get angle to provide to the shoulder motor to get
   * to a target (x,y) position
   * 
   * @param x Target length away from the base of the arm. Units in inches
   * @param y Target height away from the base of the arm. Units in inches
   * @return Angle to rotate shoulder motor
   */
  public double targetToAngleShoulder(double x, double y) {
    double a1 = Constants.shoulderLength;
    double a2 = Constants.elbowLength;
    double a3 = Constants.wristLength;

    double x1 = x - a3; // Target length minus wrist length

    double angleRad = Math.acos((Math.pow(a2, 2) - Math.pow(a1, 2) - Math.pow(x1, 2) - Math.pow(y, 2))
        / (-2 * a1 * Math.sqrt(Math.pow(x1, 2) + Math.pow(y, 2)))) + Math.atan2(y, x1);

    double angleDeg = Math.toDegrees(angleRad);
    return angleDeg;
  }

  /**
   * Method to get the angle to get angle to provide to the elbow motor to get to
   * a target (x,y) position
   * 
   * @param x Target length away from the base of the arm. Units in inches
   * @param y Target height away from the base of the arm. Units in inches
   * @return Angle to rotate elbow motor
   */
  public double targetToAngleElbow(double x, double y) {
    double a1 = Constants.shoulderLength;
    double a2 = Constants.elbowLength;
    double a3 = Constants.wristLength;

    double x1 = x - a3; // Target length minus wrist length

    double angleRad = Math.acos((Math.pow(x1, 2) + Math.pow(y, 2) - Math.pow(a1, 2) - Math.pow(a2, 2)) / (-2 * a1 * a2))
        - Math.PI;

    double angleDeg = Math.toDegrees(angleRad);
    return angleDeg;
  }

  /**
   * Method to set the angle of individual motors on the arm
   * 
   * @param jointTalon Which joint arm motor to use
   * @param degrees    Target angle in degrees
   */
  public void setToAngle(TalonSRX jointTalon, double degrees) {
    double sprocketRatio = 0;
    if (jointTalon == shoulderMotor)
      sprocketRatio = Constants.shoulderGearRatio;
    else if (jointTalon == elbowMotor)
      sprocketRatio = Constants.elbowGearRatio;
    else if (jointTalon == wristMotor)
      sprocketRatio = Constants.wristGearRatio;
    if (sprocketRatio != 0) {
      double targetPos = degrees * Constants.encoderTicksPerRotation * sprocketRatio / 360;
      setMotorPos(jointTalon, targetPos);
    }
  }

  /**
   * Uses Motion Magic to set motors to a target encoder position
   * 
   * @param talon     Joint motor to control
   * @param targetPos Target encoder position
   */
  public void setMotorPos(TalonSRX talon, double targetPos) {
    talon.set(ControlMode.MotionMagic, targetPos);
  }

  public void setMotionVelAccel(TalonSRX talon, int velocity, int accel) {
    talon.configMotionCruiseVelocity(velocity, Constants.timeout);
    talon.configMotionAcceleration(accel, Constants.timeout);
  }

  public void talonConfig(TalonSRX talon, boolean invert) {
    talon.configFactoryDefault();
    double[] pidArr = { 0, 0, 0, 0, 0 };

    if (talon == shoulderMotor)
      pidArr = Constants.shoulderPID;
    if (talon == elbowMotor)
      pidArr = Constants.elbowPID;
    if (talon == wristMotor)
      pidArr = Constants.wristPID;

    talon.setInverted(invert);
    talon.setSensorPhase(true);
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidIdx, Constants.timeout);
    talon.selectProfileSlot(RobotMap.slotIdx, RobotMap.pidIdx);
    talon.config_kF(RobotMap.slotIdx, pidArr[0], Constants.timeout);
    talon.config_kP(RobotMap.slotIdx, pidArr[1], Constants.timeout);
    talon.config_kI(RobotMap.slotIdx, pidArr[2], Constants.timeout);
    talon.config_kD(RobotMap.slotIdx, pidArr[3], Constants.timeout);
    talon.config_IntegralZone(RobotMap.slotIdx, (int) pidArr[4], Constants.timeout);
    talon.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
  }

  public void zeroEncoders() {
    shoulderMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
    elbowMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);
    wristMotor.setSelectedSensorPosition(0, RobotMap.pidIdx, Constants.timeout);

  }

  public double getEncoderPosition(TalonSRX talon) {
    return talon.getSelectedSensorPosition();
  }

  public double getEncoderVelocity(TalonSRX talon) {
    return talon.getSelectedSensorVelocity();
  }

  public double getMotorAngle(TalonSRX jointTalon) {
    double sprocketRatio = 0;
    if (jointTalon == shoulderMotor)
      sprocketRatio = Constants.shoulderGearRatio;
    else if (jointTalon == elbowMotor)
      sprocketRatio = Constants.elbowGearRatio;
    else if (jointTalon == wristMotor)
      sprocketRatio = Constants.wristGearRatio;

    return getEncoderPosition(jointTalon) / 4096 * 360 / sprocketRatio;
  }

  /**
   * @return the wristMotor
   */
  public WPI_TalonSRX getShoulderMotor() {
    return shoulderMotor;
  }

  /**
   * @return the wristMotor
   */
  public WPI_TalonSRX getElbowMotor() {
    return elbowMotor;
  }

  /**
   * @return the wristMotor
   */
  public WPI_TalonSRX getWristMotor() {
    return wristMotor;
  }

}
