/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
// import org.usfirst.frc.team4669.robot.commands.OpenGrabber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class GearShift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid doubleAction = new DoubleSolenoid(RobotMap.SOLENOID_FORWARD, RobotMap.SOLENOID_BACKWARD);
  private Compressor compressor = new Compressor();

  public void highGear() {
    doubleAction.set(DoubleSolenoid.Value.kForward);
  }

  public void lowGear() {
    doubleAction.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean getHighGear() {
    if (doubleAction.get() == DoubleSolenoid.Value.kForward) {
      return true;
    }
    return false;
  }

  public void startCompressor(){
    compressor.start();
  }

  public void stopCompressor(){
    compressor.stop();
  }

  public boolean isCompressorRunning(){
    return compressor.enabled();
  }

  public boolean isCompressorClosedLoopOn(){
    return compressor.getClosedLoopControl();
  }

  public boolean isPressureLow(){
    return compressor.getPressureSwitchValue();
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // Not using default commands
  }

}