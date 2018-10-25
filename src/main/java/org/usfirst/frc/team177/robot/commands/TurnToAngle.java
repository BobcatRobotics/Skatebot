/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team177.robot.commands;

import org.usfirst.frc.team177.lib.SmartPID;
import org.usfirst.frc.team177.robot.OI;

public class TurnToAngle extends DriveCommand {
	private double newAngle = 0.0;
	private double tolerance = 1.0;  // in degrees
	
	/// XXXXX TODO:: Make these configurable
	private double pidP = 0.02 ;
	private double pidI = 0.003;
	private double pidD = 0.0;
	private SmartPID pid = new SmartPID(0.0,pidP,pidI,pidD);
		
	private TurnToAngle() {
		super();
		newAngle = OI.gyro.getYaw();
	}

	public TurnToAngle(double angleToTurn) {
		this();
		OI.driveTrain.setLeftPower(RobotConstants.INITIAL_LEFT_POWER_FORWARD);
		OI.driveTrain.setRightPower(RobotConstants.INITIAL_RIGHT_POWER_FORWARD);
		
		OI.gyro.setGyroPID(pid);
		OI.gyro.setToleranceDegrees(tolerance);
		OI.gyro.initTurnController();
		
		double startingAngle = OI.gyro.getYaw();
		double angle = startingAngle + angleToTurn;
		newAngle = adjustAngleChange(angle);
		OI.gyro.turnToAngle(newAngle);
	}

	@Override
	protected void execute() {
		// Turning
		double rate = OI.gyro.getRotateToAngleRate();
		OI.driveTrain.drive(rate * -1.0, rate);
	}

	@Override
	protected boolean isFinished() {
		boolean stop = false;
		// Stop if new angle is within tolerance of requested angle
		double currentYaw = OI.gyro.getYaw();
		if (Math.abs(currentYaw - newAngle ) < tolerance) {
			stop = true;
		}
		return stop;
	}

	@Override
	protected void end() {
		OI.gyro.stopTurn();
		OI.driveTrain.stop();
	}

	// Adjust so that -180 <= angle <= 180
	private double adjustAngleChange(double angle) {
		double newAngle = angle;
		if (angle > 180) {
			newAngle -= 360;
		} else if (angle < -180) {
			newAngle += 360;
		}
		return newAngle;
	}
}
