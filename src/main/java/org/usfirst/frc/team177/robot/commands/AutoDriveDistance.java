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
public class AutoDriveDistance extends AutoDrive {

	@SuppressWarnings("unused")
	private AutoDriveDistance() {
	}

	public AutoDriveDistance(double distance) {
		super(distance,null);
	}

	public AutoDriveDistance(double distance,boolean driveStraight) {
		super(distance,null);
		driveStraight(driveStraight);
	}

	public AutoDriveDistance(double distance, double timeToStop) {
		super(distance,null,timeToStop);
	}
	
	public AutoDriveDistance(double distance, double timeToStop,boolean driveStraight) {
		super(distance,null,timeToStop);
		driveStraight(driveStraight);
	}
}
