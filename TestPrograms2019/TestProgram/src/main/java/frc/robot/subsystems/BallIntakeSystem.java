package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.BallIntakeCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;


/**
 *
 */
public class BallIntakeSystem extends Subsystem {
	
    private WPI_TalonSRX intakeMotor;
    private WPI_TalonSRX jawMotor;
	

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	

	public BallIntakeSystem(){
        super();
        	
		intakeMotor =  new WPI_TalonSRX(RobotMap.INTAKE_MOTOR);
		jawMotor =  new WPI_TalonSRX(RobotMap.JAW_MOTOR);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new BallIntakeCommand());
    }
    
    public void intake(){
//    	double speed = SmartDashboard.getNumber("CubeIntakeVel", 2500);
        intakeMotor.set(ControlMode.PercentOutput,0.4);
//    	leftIntakeMotor.set(ControlMode.Velocity,speed);
//    	rightIntakeMotor.set(ControlMode.Velocity,speed);
    }
    
    public void openJaw(){
        jawMotor.set(ControlMode.PercentOutput,0.25);
    }

    public void closeJaw(){
        jawMotor.set(ControlMode.PercentOutput,-0.15);
    }

    public void releaseBall(){
        intakeMotor.set(ControlMode.PercentOutput,-0.4);
    }
    
    public void set(double output){
    	jawMotor.set(ControlMode.PercentOutput,output);
    	intakeMotor.set(ControlMode.PercentOutput,output);
    }
    
    public void stopIntake(){
		intakeMotor.set(0);
    }

    public void stopJaw(){
        jawMotor.set(0);
    }

}