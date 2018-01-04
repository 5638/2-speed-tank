package org.usfirst.frc5638.shiftyTenk.subsystems;

import org.usfirst.frc5638.shiftyTenk.Robot;
import org.usfirst.frc5638.shiftyTenk.RobotMap;

import com.ctre.MotorControl.SmartMotorController.TalonControlMode;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class encoder extends Subsystem {
	public static double kP = 0.0;
	public static double kI = 0.0;
	public static double kD = 0.0;
	public static double kF = 0.0;
	int izone = 100;	//encoder ticks
	double ramprate = 6;	//volts/sec
	int profile = 0; //0 or 1
	private double m_outputleft;
	private double m_outputright;
	public double setpointleft;
	public double setpointright;
    // Initialize your subsystem here
    public encoder() {
    	RobotMap.driveTrainleft.changeControlMode(TalonControlMode.Position);
    	RobotMap.driveTrainright.changeControlMode(TalonControlMode.Position);
    	
    	RobotMap.driveTrainleft.setSetpoint(setpointleft);
    	RobotMap.driveTrainright.setSetpoint(setpointright);
    	
    	RobotMap.driveTrainleft.setPID(kP, kI, kD, kF, izone, ramprate, profile); //PID Left
    	RobotMap.driveTrainright.setPID(kP, kI, kD, kF, izone, ramprate, profile);//PID right
    	
    	
    	m_outputleft = RobotMap.driveTrainleft.pidGet();	//gets PID output for left side.
    	m_outputright = RobotMap.driveTrainright.pidGet();	//gets PID output for right side.
    	
    	
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput(double output) {
		
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	return output;
    	
    }

    protected void usePIDOutput(double outputleft, double outputright) {
    	outputleft = m_outputleft;
    	outputright = m_outputright;
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	RobotMap.driveTrainleft.set(outputleft);
    	RobotMap.driveTrainright.set(outputright);
    }
    
    public void done() {
    	RobotMap.driveTrainleft.set(0);
    	RobotMap.driveTrainright.set(0);
    	RobotMap.driveTrainleft.changeControlMode(TalonControlMode.PercentVbus);
    	RobotMap.driveTrainright.changeControlMode(TalonControlMode.PercentVbus);
    }
}
