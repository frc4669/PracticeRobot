package org.usfirst.frc.team4669.robot.subsystems;

import org.usfirst.frc.team4669.robot.Constants;
import org.usfirst.frc.team4669.robot.RobotMap;
import org.usfirst.frc.team4669.robot.commands.CurvatureDrive;
//import org.usfirst.frc.team4669.robot.commands.TankDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	private AHRS imu;
	
	private double lastLinearAccelX = 0.0;
	private double lastLinearAccelY = 0.0;
	
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
    	
		
    	bottomRightMotor.setInverted(false);
		topRightMotor.setInverted(false);
    	bottomLeftMotor.setInverted(false);
		topLeftMotor.setInverted(false);
    	
		topRightMotor.setSensorPhase(true);
		topLeftMotor.setSensorPhase(true);
    	
    	leftMotorGroup = new SpeedControllerGroup(topLeftMotor, bottomLeftMotor);
    	rightMotorGroup = new SpeedControllerGroup(topRightMotor, bottomRightMotor);
    	
    	leftMotorGroup.setInverted(false);
    	rightMotorGroup.setInverted(false);
    	
    	drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    	drive.setSafetyEnabled(false);
    	
    	topRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
    	topLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
    	
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
		
		/* set closed loop gains in slot0 - see documentation */
		topRightMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		topRightMotor.config_kF(Constants.kSlotIdx, Constants.kF, Constants.kTimeoutMs);
		topRightMotor.config_kP(Constants.kSlotIdx, Constants.kP, Constants.kTimeoutMs);
		topRightMotor.config_kI(Constants.kSlotIdx, Constants.kI, Constants.kTimeoutMs);
		topRightMotor.config_kD(Constants.kSlotIdx, Constants.kD, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		topRightMotor.configMotionCruiseVelocity(0, Constants.kTimeoutMs);
		topRightMotor.configMotionAcceleration(0, Constants.kTimeoutMs); 
		
		/* set closed loop gains in slot0 - see documentation */
		topLeftMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		topLeftMotor.config_kF(Constants.kSlotIdx, Constants.kF, Constants.kTimeoutMs);
		topLeftMotor.config_kP(Constants.kSlotIdx, Constants.kP, Constants.kTimeoutMs);
		topLeftMotor.config_kI(Constants.kSlotIdx, Constants.kI, Constants.kTimeoutMs);
		topLeftMotor.config_kD(Constants.kSlotIdx, Constants.kD, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		topLeftMotor.configMotionCruiseVelocity(Constants.cruiseVel, Constants.kTimeoutMs);
		topLeftMotor.configMotionAcceleration(Constants.accel, Constants.kTimeoutMs); 
    }
    
    public void stop(){
    	tankDrive(0,0,false);
    }
    
	
//	public boolean hasCollided(){
//		boolean collisionDetected = false;
//        
//        double currLinearAccelX = imu.getWorldLinearAccelX();
//        double currentJerkX = currLinearAccelX - lastLinearAccelX;
//        lastLinearAccelX = currLinearAccelX;
//        double currLinearAccelY = imu.getWorldLinearAccelY();
//        double currentJerkY = currLinearAccelY - lastLinearAccelY;
//        lastLinearAccelY = currLinearAccelY;
//        
//        if ( ( Math.abs(currentJerkX) > RobotMap.kCollisionThresholdDeltaG ) ||
//             ( Math.abs(currentJerkY) > RobotMap.kCollisionThresholdDeltaG) ) {
//            collisionDetected = true;
//        }
//        SmartDashboard.putBoolean(  "CollisionDetected", collisionDetected);
//        
//        return collisionDetected;
//	}
    
    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn){
    	drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
    }
    
    public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs){
    	drive.arcadeDrive(xSpeed, zRotation, squaredInputs);
    }
    
    public void tankDrive(double leftSpeed, double rightSpeed,boolean squaredInputs){
    	drive.tankDrive(leftSpeed, rightSpeed,squaredInputs);
    }
    
    
    public int getLeftEncoder() {
    	return topLeftMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx);
    }
    
    public int getRightEncoder() {
    	return topRightMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx);
    }
    
    public int getLeftEncoderSpeed() {
    	return topLeftMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx);
    }
    
    public int getRightEncoderSpeed() {
    	return topRightMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx);
    }
    
    public void setLeftSelectedSensorPosition(int sensorPos) {
    	topLeftMotor.setSelectedSensorPosition(sensorPos, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
    
    public void setRightSelectedSensorPosition(int sensorPos) {
    	topRightMotor.setSelectedSensorPosition(sensorPos, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
    
    /**
     * @param distance The distance for robot to travel in encoder units
     */
    public void driveMotionMagic(double distance) {
    	topLeftMotor.set(ControlMode.MotionMagic, distance); 
    	topRightMotor.set(ControlMode.MotionMagic, distance); 
    }
    
    public void setMotionVelAccel(int velocity,int accel) {
		topLeftMotor.configMotionCruiseVelocity(velocity,RobotMap.timeout);
		topLeftMotor.configMotionAcceleration(accel,RobotMap.timeout);
		topRightMotor.configMotionCruiseVelocity(velocity,RobotMap.timeout);
		topRightMotor.configMotionAcceleration(accel,RobotMap.timeout);
    }
    
    public boolean motionMagicIsDone(double distance) {
    	distance *= Constants.inchToEncoderUnits;
    	return (Math.abs(distance-getLeftEncoder())<=200&&Math.abs(distance-getRightEncoder())<=200);
    }
}

