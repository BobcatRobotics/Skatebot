package org.usfirst.frc.team177.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public abstract class AutoCommand extends CommandGroup {
	protected String autoGameData = "LLL";
	//private DriveStraightDistance driveDistance;

	private AutoCommand() {
		super();
	}


	public AutoCommand(String gameData) {
		this();
		if (gameData != null)
			autoGameData = gameData;
	}
	

}
