package org.usfirst.frc5638.shiftyTenk.subsystems;

import org.usfirst.frc5638.shiftyTenk.Robot;
import org.usfirst.frc5638.shiftyTenk.RobotMap;

//import com.ctre.MotorControl.SmartMotorController.TalonControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motion.*;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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
	public int setpointleft;
	public int setpointright;
	static final double kToleranceDegrees = 2.0f;
    // Initialize your subsystem here
    public encoder() {
    	RobotMap.driveTrainleft.set(ControlMode.Position, 4096); //change this? only @4096 because that is the number
    	RobotMap.driveTrainright.set(ControlMode.Position, 4096);//of ticks for the MAG encoder.
    	
    	RobotMap.driveTrainleft.setSelectedSensorPosition(setpointleft, 0, 10);
    	RobotMap.driveTrainright.setSelectedSensorPosition(setpointright, 0, 10);
    	
    	RobotMap.driveTrainleft.config_kP((int)kP, 0, 10);
    	RobotMap.driveTrainleft.config_kI((int)kI, 0, 10);
    	RobotMap.driveTrainleft.config_kD((int)kD, 0, 10);
    	RobotMap.driveTrainleft.config_kF((int)kF, 0, 10);
    	RobotMap.driveTrainleft.configClosedloopRamp(ramprate, 2);
    	RobotMap.driveTrainleft.selectProfileSlot(profile, 0);
    	RobotMap.driveTrainleft.config_IntegralZone(izone, 0, 10);  //.setPID(kP, kI, kD, kF, izone, ramprate, profile); //PID Left
    	//RobotMap.driveTrainright.setPID(kP, kI, kD, kF, izone, ramprate, profile);//PID right
    	RobotMap.driveTrainright.config_kP((int)kP, 0, 10);
    	RobotMap.driveTrainright.config_kI((int)kI, 0, 10);
    	RobotMap.driveTrainright.config_kD((int)kD, 0, 10);
    	RobotMap.driveTrainright.config_kF((int)kF, 0, 10);
    	RobotMap.driveTrainright.configClosedloopRamp(ramprate, 2);
    	RobotMap.driveTrainright.selectProfileSlot(profile, 0);
    	RobotMap.driveTrainright.config_IntegralZone(izone, 0, 10);
    	
    	
    	m_outputleft = RobotMap.driveTrainleft.getActiveTrajectoryPosition();//.pidGet();	//gets PID output for left side.
    	m_outputright = RobotMap.driveTrainright.getActiveTrajectoryPosition();//.pidGet();	//gets PID output for right side.
    	
 
    	
    	
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
    	RobotMap.driveTrainleft.set(ControlMode.PercentOutput, outputleft);
    	RobotMap.driveTrainright.set(ControlMode.PercentOutput, outputright);
    }
    
    public void done() {
    	RobotMap.driveTrainleft.set(ControlMode.PercentOutput, 0);
    	RobotMap.driveTrainright.set(ControlMode.PercentOutput, 0);
    }

}
