package org.usfirst.frc.team177.robot.commands;

import org.usfirst.frc.team177.robot.ElevatorSetPosition;

public class AutoTest extends AutoCommand {

	public AutoTest(String gameData,String fileName) {
		super(gameData);
		setAutoCommands(fileName);
	}
	
	@Override
	protected void initialize() {
		super.initialize();
	}

	private void setAutoCommands(String fileName) {
		//addSequential(new AutoDriveDistance(RobotConstants.MID_DISTANCE_1,true));
		//DriverStation.reportError("after first command", false);
		//addSequential(new AutoDriveDistanceBackwards(RobotConstants.MID_DISTANCE_1,true));
		//DriverStation.reportError("after second command", false);
		//addSequential(new TurnToAngle(RobotConstants.MID_TURN_ANGLE_1));
		//addSequential(new AutoDriveSpeed(RobotConstants.MID_MOTOR_SPEED,RobotConstants.MID_DISTANCE_3));
		addSequential(new PlaybackSpeeds(fileName)); 
		addSequential(new MoveElevatorAuto(ElevatorSetPosition.UP));
		addSequential(new EjectCube());

	}

}
