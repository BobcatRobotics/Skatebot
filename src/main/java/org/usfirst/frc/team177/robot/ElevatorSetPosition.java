package org.usfirst.frc.team177.robot;

/**
 * This class is used with the Elevator to go to a specific position
 */
public enum ElevatorSetPosition {
	NONE (0),
	UP (-1),
	DOWN(-2),
	POSITION_1 (100),
	POSITION_2 (500),
	POSITION_3 (750);
	
	private int position = 0;
	private ElevatorSetPosition(int p) {
		position = p;
	}
	
	public int getPosition () {
		return position;
	}
}
