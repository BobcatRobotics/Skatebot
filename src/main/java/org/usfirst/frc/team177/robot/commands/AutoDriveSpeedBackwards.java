package org.usfirst.frc.team177.robot.commands;

public class AutoDriveSpeedBackwards extends AutoDriveSpeed {

	public AutoDriveSpeedBackwards(double speed) {
		super(speed);
		driveBackward();
	}

	public AutoDriveSpeedBackwards(double speed, boolean driveStraight) {
		super(speed, driveStraight);
		driveBackward();
	}

	public AutoDriveSpeedBackwards(double speed, double timeToStop) {
		super(speed, timeToStop);
		driveBackward();
	}

	public AutoDriveSpeedBackwards(double speed, double timeToStop, boolean driveStraight) {
		super(speed, timeToStop, driveStraight);
		driveBackward();
	}

}
