package org.usfirst.frc5638.shiftyTenk.subsystems;

import org.usfirst.frc5638.shiftyTenk.RobotMap;
import org.usfirst.frc5638.shiftyTenk.commands.*;
import com.ctre.MotorControl.CANTalon;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class rotateAngle extends Subsystem implements PIDOutput {
	
	AHRS ahrs;
	PIDController turnController;
	double rotateToAngleRate;
	
	//Tune this vvvvv
	
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	
	//Tune this ^^^^^
	//How close the robot will attempt to get to desired heading vvvv
	
    static final double kToleranceDegrees = 2.0f;
    
    private final CANTalon left1 = RobotMap.driveTrainleft;
    private final CANTalon left2 = RobotMap.driveTrainleft1;
    private final CANTalon right1 = RobotMap.driveTrainright;
    private final CANTalon right2 = RobotMap.driveTrainright1;
    private final RobotDrive robotDrive41 = RobotMap.driveTrainrobotDrive;

    public rotateAngle() {
    	//Communications with the NavX on the MXP SPI Bus
    	try {
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    	//Communications with the NavX on the MXP SPI Bus
    	
    	turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
    	turnController.setInputRange(-180.0f, 180.0f);
    	turnController.setOutputRange(-1.0, 1.0); //motor speed?
    	turnController.setAbsoluteTolerance(kToleranceDegrees);
    	turnController.setContinuous(true);
    	
    	LiveWindow.addActuator("Rotate", "RotateSub", turnController);
    }

    public void initDefaultCommand() {
    }
    
    public void test() {
    	
    }
    
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		rotateToAngleRate = output;
	}
	
	boolean rotateToAngle = false;
	
	public void rotate(double angle) {
		turnController.setSetpoint(angle);
		rotateToAngle = true;
	}
}

