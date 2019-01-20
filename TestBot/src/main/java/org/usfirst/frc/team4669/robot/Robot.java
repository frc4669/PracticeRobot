/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot;

import org.usfirst.frc.team4669.robot.commands.*;
import org.usfirst.frc.team4669.robot.subsystems.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static NetworkTableInstance networkTableInst;
	public static NetworkTable visionTable;
	public static OI oi = new OI();
	public static F310 f310 = new F310();
	public static DriverStation driverStation;
	public static DriveTrain driveTrain;
	// public static CubeIntake intake;
	// public static Climber climber;
	public static Elevator elevator;
	public static Arm arm;

	Command autonomousCommand;
	SendableChooser<String> chooser = new SendableChooser<String>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// intake = new CubeIntake();
		// climber = new Climber();
		elevator = new Elevator();
		networkTableInst = NetworkTableInstance.getDefault();
		visionTable = networkTableInst.getTable("DataTable");
		driveTrain = new DriveTrain();
		arm = new Arm();
		driveTrain.zeroEncoders();
		driveTrain.resetGyro();
		driveTrain.calibrateGyro();
		// Sends Strings to chooser and not commands in the case of the command
		// requiring something that only occurs during auto init
		chooser.addDefault("Do Nothing", "DoNothing");
		chooser.addObject("Pathfinder", "Pathfinder");
		SmartDashboard.putData("Auto mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		driverStation = DriverStation.getInstance();
		updateSmartDashboard();
		if (autonomousCommand != null)
			autonomousCommand.cancel();

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector","Default");
		 * switch(autoSelected) { case "My Auto": autonomousCommand = new
		 * MyAutoCommand(); break; case "Default Auto": default:autonomousCommand = new
		 * ExampleCommand(); break; }
		 */

		if (chooser.getSelected().equals("DoNothing"))
			autonomousCommand = new DoNothing();
		if (chooser.getSelected().equals("Pathfinder"))
			autonomousCommand = new PathfinderTest();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}

	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Gyro Angle", driveTrain.getAngle());
		SmartDashboard.putNumber("Vision Turn Error", driveTrain.getPIDError(driveTrain.getVisionTurnController()));
		SmartDashboard.putNumber("Vision Distance Error",
				driveTrain.getPIDError(driveTrain.getVisionDistanceController()));
		SmartDashboard.putData("Gyro PID Controller", driveTrain.gyroPID);
		SmartDashboard.putData("Vision Turn PID Controller", driveTrain.getVisionTurnController());
		SmartDashboard.putData("Vision Distance PID Controller", driveTrain.getVisionDistanceController());
		SmartDashboard.putData("Align to Ball", new AlignToBall());
		SmartDashboard.putNumber("Left Encoder", Robot.driveTrain.getLeftEncoder());
		SmartDashboard.putNumber("Right Encoder", Robot.driveTrain.getRightEncoder());
	}
}