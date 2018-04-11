package org.usfirst.frc.team4669.robot.subsystems;

import edu.wpi.first.wpilibj.*;
//import org.usfirst.frc.team4669.robot.RobotMap;
//import org.usfirst.frc.team4669.robot.commands.OpenClaw;
//import org.usfirst.frc.team4669.robot.commands.CloseClaw;
import edu.wpi.first.wpilibj.command.Subsystem;

/** @author Josiah Arrington
 *  @version 0.1
 */ 

public class GrabClaw extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	//create servo object on port 0
	Servo grabServo = new Servo(0);
	
	public void initDefaultCommand() {
    }

    public void open() {
    	grabServo.set(1);
    }

    public void close() {
    	grabServo.set(-1);
    }

    public void stop() {
    	grabServo.set(0);
    }
    
}

