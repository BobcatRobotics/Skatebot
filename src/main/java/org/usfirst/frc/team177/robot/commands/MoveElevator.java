package org.usfirst.frc.team177.robot.commands;

import org.usfirst.frc.team177.robot.ElevatorState;
import org.usfirst.frc.team177.robot.OI;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public abstract class MoveElevator extends Command {

	/**
	 *  0 = initialized
	 *  1 = reached top
	 *  -1 = reached bottom
	 */
	//public int state = 0;
	
	public MoveElevator() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		OI.elevator.reset();
   }

    protected ElevatorState  checkSwitches() {
    	if (OI.elevator.upperSwitch()) {
     		return ElevatorState.LIMIT_UP;
    	}
    	else
    	if (OI.elevator.lowerSwitch()) {
      		OI.elevator.resetEncoder();
    		return ElevatorState.LIMIT_DOWN;
       	}
    	else {
    		return ElevatorState.NONE;
    	}
    } 
    
    // Called repeatedly when this Command is scheduled to run
     protected abstract void execute();

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
		OI.elevator.stop();

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		OI.elevator.stop();
    }
}
