/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team177.robot.commands;

import org.usfirst.frc.team177.robot.OI;

/**
 * Base class for autonomous driving. This class has correction routines. You can drive by
 *  	length
 *  	length, timeToStop
 *  	length, timetoStop, forward or backward
 */
public abstract class AutoDrive extends DriveCommand {
	// TODO XXXXXX Will there ever be an instance in Auto where we do not want to ramp up or ramp down
	// TODO XXXXXXX 
	
	private double initialLeftPower = 0.0;
	private double initialRightPower = 0.0;

	private boolean isTimed = false;
	private boolean driveForward = true;
	private boolean driveStraightCorrection = false;

	@SuppressWarnings("unused")
	protected AutoDrive() {
		super();
	}

	public AutoDrive(Double length,Double speed) {
		checkConstructorVariables(length,speed);
	}
	
	public AutoDrive(Double length,Double speed,Double timeToStop) {
		checkConstructorVariables(length,speed);
		setTimeout(timeToStop);
		isTimed = true;
	}

	private void checkConstructorVariables(Double length,Double speed ) {
		if (length != null) {
			distanceToDrive = length;
			initialLeftPower = RobotConstants.INITIAL_LEFT_POWER_FORWARD;
			initialRightPower = RobotConstants.INITIAL_RIGHT_POWER_FORWARD;
		}
		if (speed != null) {
			distanceToDrive = 0.0;
			initialLeftPower = speed;
			initialRightPower = speed;
		}
	}
	
	protected void driveForward () {
		this.driveForward = true;
	}
	
	protected void driveBackward () {
		this.driveForward = false;
	}
	
	protected void driveStraight(boolean straight) {
		driveStraightCorrection = straight;
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		OI.driveTrain.reset();
		OI.driveTrain.setLeftPower(initialLeftPower);
		OI.driveTrain.setRightPower(initialRightPower);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (driveStraightCorrection)
			adjustDriveStraight();
		
		// Ramp up motor speed  
		double rampScale = rampUpFactor();
		
		// Ramp down motor speed
		double distanceScale = rampDownFactor();
		
		double leftMotorSpeed = initialLeftPower*rampScale*distanceScale;
		double rightMotorSpeed = initialRightPower*rampScale*distanceScale;
		if (!driveForward) { //reverse
			leftMotorSpeed *= -1.0;
			rightMotorSpeed *= -1.0;
		}
		
		OI.driveTrain.setLeftPower(leftMotorSpeed);
		OI.driveTrain.setRightPower(rightMotorSpeed);
		
		OI.driveTrain.drive();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		boolean stop = false;
		if ((Math.abs(OI.driveTrain.getLeftDistance()) > distanceToDrive) ||
			(Math.abs(OI.driveTrain.getRightDistance()) > distanceToDrive)) {
			stop = true;
		}
		if (isTimed) {
			if (isTimedOut()) {
				stop=true;
			}
		}
		return stop;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		OI.driveTrain.stop();
	}
}
