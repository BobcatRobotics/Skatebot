package org.usfirst.frc.team177.robot.commands;

public class AutoDriveDistanceBackwards extends AutoDriveDistance {

	public AutoDriveDistanceBackwards(double distance) {
		super(distance);
		driveBackward();
	}

	public AutoDriveDistanceBackwards(double distance, boolean driveStraight) {
		super(distance, driveStraight);
		driveBackward();
	}

	public AutoDriveDistanceBackwards(double distance, double timeToStop) {
		super(distance, timeToStop);
		driveBackward();
	}

	public AutoDriveDistanceBackwards(double distance, double timeToStop, boolean driveStraight) {
		super(distance, timeToStop, driveStraight);
		driveBackward();
	}

}
