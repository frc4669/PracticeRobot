/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code
*/
/* must be accompanied by the FIRST BSD license file in the root directory of
*/
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.GearShift;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Opens the robot grabbers.
 */
public class HighGear extends InstantCommand {

    public HighGear() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.gearShifter);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        Robot.gearShifter.highGear();
    }

}