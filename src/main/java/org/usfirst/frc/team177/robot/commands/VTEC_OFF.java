package org.usfirst.frc.team177.robot.commands;

import org.usfirst.frc.team177.robot.OI;
import edu.wpi.first.wpilibj.command.Command;

public class VTEC_OFF extends Command {

    boolean shifterState = true;

    public VTEC_OFF() {

    }

    protected void initialize() {
        
        shifterState = false;
        OI.shifter.set(shifterState);
    }

    protected void execute() {

    }

    protected boolean isFinished() {

        return true;
    }

    protected void end() {

    }

    protected void interrupted() {

    }
}