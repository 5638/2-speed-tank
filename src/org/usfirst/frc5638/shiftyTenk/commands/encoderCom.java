package org.usfirst.frc5638.shiftyTenk.commands;

import org.usfirst.frc5638.shiftyTenk.Robot;
import org.usfirst.frc5638.shiftyTenk.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class encoderCom extends Command {
	
	private float m_setpoint;

    public encoderCom(float setpoint) {
    	m_setpoint = setpoint;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.encoder);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.driveTrainleft.setSetpoint(m_setpoint);
    	RobotMap.driveTrainright.setSetpoint(m_setpoint);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.encoder.done();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    	end();
    }
}
