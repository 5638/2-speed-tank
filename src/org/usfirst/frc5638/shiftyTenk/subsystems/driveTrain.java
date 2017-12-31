// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5638.shiftyTenk.subsystems;

//import org.usfirst.frc5638.shiftyTenk.Robot;
import org.usfirst.frc5638.shiftyTenk.RobotMap;
import org.usfirst.frc5638.shiftyTenk.commands.driveCom;

import com.ctre.Drive.SensoredTank;
import com.ctre.Drive.Styles;
import com.ctre.Drive.Styles.Smart;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;

//import com.ctre.MotorControl.CANTalon;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class driveTrain extends Subsystem {
	
	AHRS ahrs;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    //private final CANTalon left = RobotMap.driveTrainleft;
    //private final CANTalon left1 = RobotMap.driveTrainleft1;
    //private final CANTalon right = RobotMap.driveTrainright;
    //private final CANTalon right1 = RobotMap.driveTrainright1;
    private final SensoredTank robotDrive = RobotMap.driveTrain;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
    	setDefaultCommand(new driveCom());
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

	public void drive(Joystick xbox) {
		/*
		try {
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
        */
		// TODO Auto-generated method stub
		//double turn = Robot.oi.xbox.getRawAxis(1) * Robot.oi.xbox.getRawAxis(1);
		//double throttle = Robot.oi.xbox.getRawAxis(2 + 3);
		robotDrive.set(Smart.PercentOutput, (float) (xbox.getRawAxis(2) + -xbox.getRawAxis(3)), (float) -xbox.getRawAxis(0));
		//SmartDashboard.putNumber("Gyro Yaw", ahrs.getYaw());
	}

	public void stop() {
		// TODO Auto-generated method stub
		//robotDrive.arcadeDrive(0, 0);
		robotDrive.set(Smart.PercentOutput, 0, 0);
	}
	public void rotateAngle(double rotateValue) { 
		robotDrive.set(Smart.PercentOutput, 0, (float) rotateValue);
	}
	
	public void driveDistance(double moveValue) {
		robotDrive.set(Smart.PercentOutput, (float) moveValue, 0);
	}
}

