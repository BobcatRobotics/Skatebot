package org.usfirst.frc.team177.robot.commands;

import org.usfirst.frc.team177.robot.OI;

import edu.wpi.first.wpilibj.command.Command;

public class WinchOut extends Command {
	
	public WinchOut() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		// DriverStation.reportError("In WinchOut command", false);
		OI.climber.winchOut();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
		OI.climber.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		OI.climber.stop();
    }
}
