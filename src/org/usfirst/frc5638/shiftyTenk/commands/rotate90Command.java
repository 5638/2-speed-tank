package org.usfirst.frc5638.shiftyTenk.commands;

import org.usfirst.frc5638.shiftyTenk.Robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class rotate90Command extends Command {
	
	public AHRS ahrs;
	
    public rotate90Command() {
    	requires(Robot.rotateAngle);
    	
    	
    	try{
        	ahrs = new AHRS(SPI.Port.kMXP);
        }catch(RuntimeException ex){
        	DriverStation.reportError("ahrs problem: " + ex.getMessage(), true);
        }
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	ahrs.zeroYaw();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.rotateAngle.rotate(90);
    	System.out.println("Angle: " + ahrs.getAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
