/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team177.robot;

import org.usfirst.frc.team177.robot.commands.*;
import org.usfirst.frc.team177.robot.subsystems.*;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
//import sun.java2d.pipe.PixelFillPipe;

/**
 * 2018 Skatebot Code - Main Class
 */
public class Robot extends TimedRobot {
	
	// Declare variable for instance of Operator Interface
	public static OI oi;
	// Declare variable for instance of each Subsystem
	public static DriveTrain driveTrain;
	// Declare variable for instance of each command
	DriveWithJoysticks driveJoy;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Create instance of each subsystem:
		driveTrain = new DriveTrain();
        // Create instance of Operator Interface, not OI must be constructed after
		// subsystems. If the OI creates Commands (which it very likely will),
		// subsystems are not guaranteed to be constructed yet. Thus, their requires()
		// statements may grab null pointers. Bad news. Don't move it.
		oi = new OI();
		driveTrain.setupLeftMotor(RobotMap.driveLeftMotorFront, false);
		driveTrain.setupRightMotor(RobotMap.driveRightMotorFront, true);
		driveTrain.setupGyro();
		/* UsbCamera mainCamera = CameraServer.getInstance().startAutomaticCapture();
		mainCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
		mainCamera.setWhiteBalanceHoldCurrent();
		mainCamera.setExposureAuto(); */

		UsbCamera wideCamera = CameraServer.getInstance().startAutomaticCapture();
		//wideCamera.setVideoMode(PixelFormat.kGray, 320, 240, 30);
		// wideCamera.setWhiteBalanceHoldCurrent();
		// wideCamera.setExposureManual(4);
  	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		//Clear out the scheduler for testing, since we may have been in teleop before
		//we came int autoInit() change for real use in competition
		Scheduler.getInstance().removeAll();
	}

	@Override
	public void disabledPeriodic() {
		driveTrain.displayDriveTrainData();
		driveTrain.displayGyroData();
	}

	/**
	 * Determine which side of the switches and scales is our color
	 * Drive there and drop off a cube
	 */
	@Override
	public void autonomousInit() {
		//Clear out the scheduler for testing, since we may have been in teleop before
		//we came int autoInit() change for real use in competition
		Scheduler.getInstance().removeAll();

		// Reset Drive Train
		driveTrain.reset();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		driveTrain.displayDriveTrainData();
		driveTrain.displayGyroData();
	}

	@Override
	public void teleopInit() {		
		// Clear out the scheduler just in case
		Scheduler.getInstance().removeAll();
		// Zero the gyro setting
		driveTrain.zeroGyroYaw();
		
		driveJoy = new DriveWithJoysticks();
		driveJoy.start();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.displayDriveTrainData();
		driveTrain.displayGyroData();				
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		driveTrain.displayGyroData();
	}
}