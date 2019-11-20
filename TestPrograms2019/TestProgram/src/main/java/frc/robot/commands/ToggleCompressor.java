
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
    requires(Robot.ballIntake);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.ballIntake.isCompressorClosedLoopOn()){
      Robot.ballIntake.stopCompressor();
    } else {
      Robot.ballIntake.startCompressor();
    }
  }

}