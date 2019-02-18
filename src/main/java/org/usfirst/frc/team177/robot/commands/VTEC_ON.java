package org.usfirst.frc.team177.robot.commands;

import org.usfirst.frc.team177.robot.OI;

import edu.wpi.first.wpilibj.command.Command;

public class VTEC_ON extends Command {

    boolean shifterState = false;

    public VTEC_ON() {

    }

    protected void intialize() {

        shifterState = true;
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