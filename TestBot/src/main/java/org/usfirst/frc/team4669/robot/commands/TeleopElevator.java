// package org.usfirst.frc.team4669.robot.commands;

// import org.usfirst.frc.team4669.robot.Robot;
// import org.usfirst.frc.team4669.robot.RobotMap;
// import org.usfirst.frc.team4669.robot.misc.Constants;

// import com.ctre.phoenix.motorcontrol.ControlMode;

// import edu.wpi.first.wpilibj.command.Command;

// /**
// *
// */
// public class TeleopElevator extends Command {

// double targetPos;
// boolean motionMagicRunning = false;
// boolean magicControl = false;
// boolean positionRunning = false;
// boolean hasCubeRunning = false;

// public TeleopElevator() {
// // Use requires() here to declare subsystem dependencies
// // eg. requires(chassis);
// requires(Robot.elevator);
// }

// // Called just before this Command runs the first time
// protected void initialize() {
// // Robot.elevator.zeroEncoders();
// motionMagicRunning = false;
// }

// // Called repeatedly when this Command is scheduled to run
// protected void execute() {
// if (Robot.elevator.getForwardLimit()) {
// Robot.elevator.zeroEncoders();
// }
// if (Robot.oi.getLeftRawButton(1)) {
// Robot.elevator.stop();
// motionMagicRunning = false;
// } else {
// // if (Robot.cubeIntake.hasCube() && !motionMagicRunning && !hasCubeRunning){
// // Robot.elevator.setHeight(RobotMap.elevatorExchange/2);
// // targetPos = RobotMap.elevatorExchange/2;
// // motionMagicRunning = true;
// // hasCubeRunning = true;
// // }
// // else if(!Robot.cubeIntake.hasCube() && hasCubeRunning){
// // hasCubeRunning = false;
// // }
// if (Robot.oi.getLeftRawButton(2) && !motionMagicRunning) {
// Robot.elevator.groundHeight();
// targetPos = 0;
// motionMagicRunning = true;
// } else if (Robot.oi.getLeftRawButton(5) && !motionMagicRunning) {
// Robot.elevator.setHeight(Constants.elevatorSwitch);
// targetPos = Constants.elevatorSwitch;
// motionMagicRunning = true;
// } else if (Robot.oi.getLeftRawButton(7) && !motionMagicRunning) {
// Robot.elevator.setHeight(Constants.elevatorExchange);
// targetPos = Constants.elevatorExchange;
// motionMagicRunning = true;
// } else if (Robot.oi.getLeftRawButton(4) && !motionMagicRunning) {
// Robot.elevator.setHeight(Constants.elevatorLift);
// targetPos = Constants.elevatorLift;
// motionMagicRunning = true;
// } else if (Robot.oi.getLeftRawButton(6) && !motionMagicRunning) {
// Robot.elevator.setHeight(Constants.elevatorScaleMid);
// targetPos = Constants.elevatorScaleMid;
// motionMagicRunning = true;
// } else if (Robot.oi.getLeftRawButton(3) && !motionMagicRunning) {
// Robot.elevator.setHeight(Constants.elevatorMax);
// targetPos = Constants.elevatorMax;
// motionMagicRunning = true;
// } else if (motionMagicRunning && Math.abs(targetPos -
// Robot.elevator.getEncoderPos()) < 200
// || Robot.elevator.getReverseLimit()) {
// Robot.elevator.zeroVelocity();
// motionMagicRunning = false;
// }

// else if (!motionMagicRunning) {
// double initialPos = 0;
// if (Robot.oi.leftY() == 0) {
// // Robot.elevator.zeroVelocity();
// magicControl = false;
// }
// if (Robot.oi.leftY() != 0 && !magicControl) {
// // Robot.elevator.percentOutput(0.55 * Robot.oi.leftY());
// initialPos = Robot.elevator.getEncoderPos();
// magicControl = true;
// } else if (Robot.oi.leftY() != 0 && magicControl) {
// Robot.elevator.controlHeight(Robot.oi.leftY(), initialPos);
// }
// }
// }
// }

// // Make this return true when this Command no longer needs to run execute()
// protected boolean isFinished() {
// return false;
// }

// // Called once after isFinished returns true
// protected void end() {
// }

// // Called when another command which requires one or more of the same
// // subsystems is scheduled to run
// protected void interrupted() {

// }
// }
