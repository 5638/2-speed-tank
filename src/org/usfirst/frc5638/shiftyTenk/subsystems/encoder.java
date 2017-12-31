package org.usfirst.frc5638.shiftyTenk.subsystems;

import org.usfirst.frc5638.shiftyTenk.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class encoder extends Subsystem {
	public double setpoint;
	public static double kP = 0.0;
	public static double kI = 0.0;
	public static double kD = 0.0;
	public static double kF = 0.0;
	int izone = 100;	//encoder ticks
	double ramprate = 6;	//volts/sec
	int profile = 0; //0 or 1

    // Initialize your subsystem here
    public encoder() {
    	RobotMap.driveTrainleft.setPID(kP, kI, kD, kF, izone, ramprate, profile);
    	RobotMap.driveTrainright.setPID(kP, kI, kD, kF, izone, ramprate, profile);
    	
    	RobotMap.driveTrain.SetPosition(1000);
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return 0.0;
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }
}
