/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team177.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team177.robot.OI;
import org.usfirst.frc.team177.robot.Robot;
import org.usfirst.frc.team177.robot.RobotMap;

public class DriveWithJoysticks extends Command {
	
	private double previousPos = 0.0;

	public DriveWithJoysticks() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.driveTrain.reset();
	}
	
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}

	@Override
	protected void execute() {
		// Driving
		//double left = OI.leftStick.getRawAxis(Joystick.AxisType.kY.value);
		//double right = OI.rightStick.getRawAxis(Joystick.AxisType.kY.value);
		double left = OI.gamePad.getRawAxis(RobotMap.gamePadLeftPwrStick);
		double right = OI.gamePad.getRawAxis(RobotMap.gamePadRightPwrStick);
		System.out.println(left);
		
		if (Math.abs(left) < .02) {

			left = 0.0;
		}

		if (Math.abs(right) < .02) {

			right = 0.0;
		}

		// if (Math.abs((right - previousPos) / (Math.pow(Timer.getFPGATimestamp(), 2))) > /*To be replaced*/.5
		// || Math.abs((left - previousPos) / (Math.pow(Timer.getFPGATimestamp(), 2))) > /*To be replaced*/.5 ) {
			
		// 	new VTEC_ON();
		// } else {

		// 	new VTEC_OFF();
		// }

		if (Math.abs(right) > .85 || Math.abs(left) > .85) {

			new VTEC_ON();
		} else {

			new VTEC_OFF();
		}
		Robot.driveTrain.setLeftPower(left);
		Robot.driveTrain.setRightPower(right);
		Robot.driveTrain.drive();
		// if (right >= left) {
		// 	previousPos = right;
		// } else {
		// 	previousPos = left;
		// }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}
	
	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.driveTrain.stop();
	}
}
