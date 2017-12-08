package org.usfirst.frc5638.shiftyTenk.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc5638.shiftyTenk.Robot;
import org.usfirst.frc5638.shiftyTenk.commands.AutonomousCommand;

import com.kauailabs.navx.frc.AHRS;
/**
 *
 */
public class rotateAngle extends Subsystem {
	public AHRS ahrs;
	
	private static final double kAngleSetpoint = 0.0;
	private static final double kP = .005; // propotional turning constant
	private static final double kI = .1;
	private static final double kD = 0;
	// gyro calibration constant, may need to be adjusted;
	// gyro value of 360 is set to correspond to one full revolution
	
	public void initDefaultCommand() {
		setDefaultCommand(new AutonomousCommand());
		
		try{
        	ahrs = new AHRS(SPI.Port.kMXP);
        }catch(RuntimeException ex){
        	DriverStation.reportError("ahrs problem: " + ex.getMessage(), true);
        }
    }
	
	public void robotInit() {
		
		ahrs.zeroYaw();
	}


	

	/**
	 * The motor speed is set from the joystick while the RobotDrive turning
	 * value is assigned from the error between the setpoint and the gyro angle.
	 */
	public void PIDTurn() {
		double turningValue = ((kAngleSetpoint - ahrs.getYaw()) * kP); //* kI;
		// Invert the direction of the turn if we are going backwards
		Robot.driveTrain.rotateAngle(turningValue);
		SmartDashboard.putNumber("angle", ahrs.getAngle());
	}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.


}

