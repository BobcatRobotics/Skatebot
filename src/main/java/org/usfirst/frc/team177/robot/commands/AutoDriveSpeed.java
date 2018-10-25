/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team177.robot.commands;

/**
 * This will drive the with a certain speed
 */
public class AutoDriveSpeed extends AutoDrive {

	@SuppressWarnings("unused")
	private AutoDriveSpeed() {
	}

	public AutoDriveSpeed(double speed) {
		super(null,speed);
	}

	public AutoDriveSpeed(double speed,boolean driveStraight) {
		super(null,speed);
		driveStraight(driveStraight);
	}

	public AutoDriveSpeed(double speed, double timeToStop) {
		super(null,speed,timeToStop);
	}
	
	public AutoDriveSpeed(double speed, double timeToStop,boolean driveStraight) {
		super(null,speed,timeToStop);
		driveStraight(driveStraight);
	}
}
