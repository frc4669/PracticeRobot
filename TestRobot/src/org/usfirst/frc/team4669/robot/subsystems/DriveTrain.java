package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.CurvatureDrive;
//import org.usfirst.frc.team4669.robot.commands.TankDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/** @author Khanh Hoang
 *  @version 0.1
 */ 

public class DriveTrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private WPI_TalonSRX bottomRightMotor;
	private WPI_TalonSRX topRightMotor;
	private WPI_TalonSRX bottomLeftMotor;
	private WPI_TalonSRX topLeftMotor;
	private SpeedControllerGroup leftMotorGroup;
	private SpeedControllerGroup rightMotorGroup;
	public DifferentialDrive drive;
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
//        setDefaultCommand(new TankDrive());
        setDefaultCommand(new CurvatureDrive());
    }
    
    public DriveTrain() {
    	super();
    	bottomRightMotor = new WPI_TalonSRX(RobotMap.bottomRightMotor);
    	topRightMotor = new WPI_TalonSRX(RobotMap.topRightMotor);
    	bottomLeftMotor = new WPI_TalonSRX(RobotMap.bottomLeftMotor);
    	topLeftMotor = new WPI_TalonSRX(RobotMap.topLeftMotor);
		
    	bottomRightMotor.setInverted(true);
		topRightMotor.setInverted(true);
    	bottomLeftMotor.setInverted(false);
		topLeftMotor.setInverted(false);
    	
		topRightMotor.setSensorPhase(true);
		topLeftMotor.setSensorPhase(true);
    	
    	leftMotorGroup = new SpeedControllerGroup(topLeftMotor, bottomLeftMotor);
    	rightMotorGroup = new SpeedControllerGroup(topRightMotor, bottomRightMotor);
    	
    	leftMotorGroup.setInverted(false);
    	rightMotorGroup.setInverted(true);
    	
    	drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

		/* set the peak and nominal outputs, 1 means full */
		topRightMotor.configNominalOutputForward(0, RobotMap.timeout);
		topRightMotor.configNominalOutputReverse(0, RobotMap.timeout);
		topRightMotor.configPeakOutputForward(1, RobotMap.timeout);
		topRightMotor.configPeakOutputReverse(-1, RobotMap.timeout);
		
		topLeftMotor.configNominalOutputForward(0, RobotMap.timeout);
		topLeftMotor.configNominalOutputReverse(0, RobotMap.timeout);
		topLeftMotor.configPeakOutputForward(1, RobotMap.timeout);
		topLeftMotor.configPeakOutputReverse(-1, RobotMap.timeout);
		
		
		//Set Current limit 
		topRightMotor.configContinuousCurrentLimit(20, RobotMap.timeout);
		topRightMotor.configPeakCurrentLimit(22, RobotMap.timeout);
		topRightMotor.configPeakCurrentDuration(50, RobotMap.timeout);
		topRightMotor.enableCurrentLimit(true);
		
		topLeftMotor.configContinuousCurrentLimit(20, RobotMap.timeout);
		topLeftMotor.configPeakCurrentLimit(22, RobotMap.timeout);
		topLeftMotor.configPeakCurrentDuration(50, RobotMap.timeout);
		topLeftMotor.enableCurrentLimit(true);
		
		//Setting bottom motors to follow top
		bottomRightMotor.set(ControlMode.Follower, topRightMotor.getDeviceID());
		bottomLeftMotor.set(ControlMode.Follower, topLeftMotor.getDeviceID());
    }
    
    public void stop(){
    	bottomLeftMotor.set(0);
    	bottomRightMotor.set(0);
    	topLeftMotor.set(0);
    	topRightMotor.set(0);
    }
    
//  public void driveForward(double leftVBus, double rightVBus){
//    	bottomRightMotor.set(ControlMode.PercentOutput,rightVBus);
//    	topLeftMotor.set(ControlMode.PercentOutput,-leftVBus); //should probably invert left motor instead
//  }
    
    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn){
    	drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
    }
    
}

