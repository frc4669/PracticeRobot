// package org.usfirst.frc.team4669.robot.subsystems;

// import org.usfirst.frc.team4669.robot.Robot;
// import org.usfirst.frc.team4669.robot.RobotMap;
// import org.usfirst.frc.team4669.robot.commands.TeleopClimber;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.BuiltInAccelerometer;
// import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.interfaces.Accelerometer;

// /**
// *
// */
// public class Climber extends Subsystem {

// private WPI_TalonSRX centerClimber;
// private WPI_TalonSRX rightClimber;
// private WPI_TalonSRX leftClimber;
// public Accelerometer accel;

// // Put methods for controlling this subsystem
// // here. Call these from Commands.

// public Climber() {
// super();
// centerClimber = new WPI_TalonSRX(RobotMap.centerClimber);
// rightClimber = new WPI_TalonSRX(RobotMap.rightClimber);
// leftClimber = new WPI_TalonSRX(RobotMap.leftClimber);

// accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);

// }

// public void initDefaultCommand() {
// // Set the default command for a subsystem here.
// // setDefaultCommand(new MySpecialCommand());
// setDefaultCommand(new TeleopClimber());
// }

// public void climbAll() {
// rightClimber.set(0.7);
// leftClimber.set(0.7);
// }

// public double getAccelX() {
// return accel.getX();
// }

// public double getAccelY() {
// return accel.getY();
// }

// public double getAccelZ() {
// return accel.getZ();
// }

// public void stop() {
// centerClimber.set(0);
// rightClimber.set(0);
// leftClimber.set(0);
// }
// }
