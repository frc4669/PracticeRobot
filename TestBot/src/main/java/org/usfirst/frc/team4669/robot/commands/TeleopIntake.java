// package org.usfirst.frc.team4669.robot.commands;

// import org.usfirst.frc.team4669.robot.F310;
// import org.usfirst.frc.team4669.robot.Robot;

// import edu.wpi.first.wpilibj.command.Command;

// /**
// *
// */
// public class TeleopIntake extends Command {

// public TeleopIntake() {
// // Use requires() here to declare subsystem dependencies
// // eg. requires(chassis);
// requires(Robot.intake);
// }

// // Called just before this Command runs the first time
// protected void initialize() {
// Robot.intake.stopIntake(); // Makes sure the intake isn't running first
// }

// // Called repeatedly when this Command is scheduled to run
// protected void execute() {
// if (Robot.f310.getButton(F310.leftShoulderButton))
// Robot.intake.releaseArms();
// else
// Robot.intake.stopServo();
// // if(Robot.f310.getButton(F310.blueButton)){
// // Robot.intake.turnCubeLeft();
// // } else if(Robot.f310.getButton(F310.redButton)){
// // Robot.intake.turnCubeRight();
// // }
// if (Robot.f310.getButton(F310.greenButton)) {
// // Attempt at autorotate and intake
// if ((Robot.intake.getLeftDistance() > 0.7) &&
// !(Robot.intake.getRightDistance() > 0.7)) {
// // If cube is recognized on left but not right, turn right motors
// Robot.intake.stopLeft();
// Robot.intake.turnCubeRight();
// } else if (!(Robot.intake.getLeftDistance() > 1) &&
// (Robot.intake.getRightDistance() > 1)) {
// // If cube is recognized on right but not left, turn left motors
// Robot.intake.stopRight();
// Robot.intake.turnCubeLeft();
// } else
// Robot.intake.intake();
// } else if (Robot.f310.getButton(F310.orangeButton)) {
// Robot.intake.releaseCube();
// } else if (Robot.f310.getRightTrigger() != 0) {
// Robot.intake.set(-0.7, -0.7);
// } else {
// Robot.intake.stopIntake();
// }
// }

// // Make this return true when this Command no longer needs to run execute()
// protected boolean isFinished() {
// return false;
// }

// // Called once after isFinished returns true
// protected void end() {
// Robot.intake.stopIntake();
// }

// // Called when another command which requires one or more of the same
// // subsystems is scheduled to run
// protected void interrupted() {
// }
// }
