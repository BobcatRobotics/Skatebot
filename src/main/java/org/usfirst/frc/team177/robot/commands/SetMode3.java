/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team177.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team177.robot.Robot;

public class SetMode3 extends Command {
	
	public SetMode3() {
		// Use requires() here to declare subsystem dependencies
		// Let's try without using the requires, since we just jumping in and
		// changing the drivetrain mode of control
		//requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.driveTrain.setmode3();
	}
	
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}

	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
	}
	
	// Called once after isFinished returns true
	@Override
	protected void end() {
	}
}
