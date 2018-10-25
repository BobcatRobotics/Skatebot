package org.usfirst.frc.team177.robot;

/**
 * This class is used with the Elevator to go to a specific position
 */
public enum ElevatorState {
	/**
	 *  0 = initialized
	 *  1 = reached top
	 *  -1 = reached bottom
	 */
	NONE (0),
	LIMIT_UP (1),
	LIMIT_DOWN(-1);
	
	private int position = 0;
	private ElevatorState(int p) {
		position = p;
	}
	
	public int getPosition () {
		return position;
	}
}
