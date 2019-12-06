package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.ElevatorCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {
	
	private WPI_TalonSRX elevatorMotor;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public Elevator(){
        super();

        elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR);

	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ElevatorCommand());
    }
    
    public void elevatorUp(){
        if(!getForwardLimit()){
            elevatorMotor.set(ControlMode.PercentOutput,0.5);
        }
    }

    public void elevatorDown(){
        if(!getReverseLimit()){
            elevatorMotor.set(ControlMode.PercentOutput,-0.3);
        }
    }
  
    public void stop()  {
    	elevatorMotor.set(0);
    }

    public double getMotorOutput(){
    	return elevatorMotor.getMotorOutputPercent();
    }
    
    public boolean getForwardLimit(){
    	return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }
    
    public boolean getReverseLimit(){
    	return elevatorMotor.getSensorCollection().isRevLimitSwitchClosed();
    }
    
    public void zeroVelocity(){
    	elevatorMotor.set(ControlMode.Velocity, 0);
    }
}