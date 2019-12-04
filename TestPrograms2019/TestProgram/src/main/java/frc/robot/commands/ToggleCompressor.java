
package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class ToggleCompressor extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ToggleCompressor() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.gearShifter);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.gearShifter.isCompressorClosedLoopOn()){
      Robot.gearShifter.stopCompressor();
    } else {
      Robot.gearShifter.startCompressor();
    }
  }

}