package org.usfirst.frc5638.shiftyTenk.subsystems;

import org.usfirst.frc5638.shiftyTenk.Robot;
import org.usfirst.frc5638.shiftyTenk.RobotMap;
import org.usfirst.frc5638.shiftyTenk.commands.*;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class rotateAngle extends PIDSubsystem implements PIDOutput {
	
	AHRS ahrs;
	public PIDController turnController;
	double rotateToAngleRate;
	double currentRotationRate;
	
	//Tune this vvvvv
	
	static final double kP = 10;
	static final double kI = 0.0;		//BAD VALUES
	static final double kD = 0.00;
	static final double kF = .00;
	
	//Tune this ^^^^^
	//How close the robot will attempt to get to desired heading vvvv
	
    static final double kToleranceDegrees = 2.0f;


    public rotateAngle() {
    	super("rotateAngle", kP, kI, kD);
    	setAbsoluteTolerance(kToleranceDegrees);
    	getPIDController().setContinuous(true);
    	LiveWindow.addActuator("rotateAngle", "PIDSubsystem Controller", getPIDController());
    	
    	
    	//Communications with the NavX on the MXP SPI Bus
    	try {
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    	LiveWindow.addSensor("rotateAngle", "Gyro NavX", ahrs);
    	
    	//Communications with the NavX on the MXP SPI Bus
    	/*
    	turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
    	turnController.setPID(kP, kI, kD, kF);
    	turnController.setInputRange(-180.0f, 180.0f);
    	turnController.setOutputRange(-.5, .5); //motor speed?
    	turnController.setAbsoluteTolerance(kToleranceDegrees);
    	turnController.setContinuous(true);
    	
    	SmartDashboard.putNumber("Gyro", ahrs.getYaw());
		SmartDashboard.putNumber("p", turnController.getP());
    	SmartDashboard.putNumber("i", turnController.getI());
    	SmartDashboard.putNumber("d", turnController.getD());
    	SmartDashboard.putNumber("f", turnController.getF());
    	SmartDashboard.putNumber("error", turnController.getError());
    	SmartDashboard.putNumber("setpoint", turnController.getSetpoint());
    	SmartDashboard.putNumber("avg. error all", turnController.getAvgError());
    	*/
    	
    }

    public void initDefaultCommand() {
    }
    
    public void test() {
    	
    }
    
	

	
/*
	
	public void rotate0() {
		turnController.setSetpoint(0);
		Robot.driveTrain.rotateAngle(turnController.get());
		turnController.enable();
		
		SmartDashboard.putNumber("Gyro", ahrs.getYaw());
		SmartDashboard.putNumber("p", turnController.getP());
    	SmartDashboard.putNumber("i", turnController.getI());
    	SmartDashboard.putNumber("d", turnController.getD());
    	SmartDashboard.putNumber("f", turnController.getF());
    	SmartDashboard.putNumber("error", turnController.getError());
    	SmartDashboard.putNumber("setpoint", turnController.getSetpoint());
    	SmartDashboard.putNumber("avg. error", turnController.getAvgError());
	}
	public void rotate90() {
		turnController.setSetpoint(90);
		Robot.driveTrain.rotateAngle(turnController.get());
		turnController.enable();
		
		SmartDashboard.putNumber("Gyro", ahrs.getYaw());
		SmartDashboard.putNumber("p", turnController.getP());
    	SmartDashboard.putNumber("i", turnController.getI());
    	SmartDashboard.putNumber("d", turnController.getD());
    	SmartDashboard.putNumber("f", turnController.getF());
    	SmartDashboard.putNumber("error", turnController.getError());
    	SmartDashboard.putNumber("setpoint", turnController.getSetpoint());
    	SmartDashboard.putNumber("avg. error", turnController.getAvgError());
	}
*/
	public void zeroGyro() {
		ahrs.reset();
	}

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return ahrs.getYaw();
	}
	
	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		Robot.driveTrain.rotateAngle(output);
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}
}

