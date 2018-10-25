package org.usfirst.frc.team177.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Climber {
	
	/* Elevator Motors */
	//private WPI_VictorSPX climberMotor1; // This is the arm motor
	//private WPI_VictorSPX climberMotor2; // This is one of the winch motors
	//private WPI_VictorSPX climberMotor3; // This is the other winch motor
	
	//private double rotateUpSpeed   = 0.3;
	//private double rotateDownSpeed = -0.3;
	private double winchInSpeed   = 1.0;
	private double winchOutSpeed = -1.0;
	
	public Climber() {
		super();
		//climberMotor1 = new WPI_VictorSPX(RobotMap.climberMotor1canID);
		//climberMotor2 = new WPI_VictorSPX(RobotMap.climberMotor2canID);
		//climberMotor3 = new WPI_VictorSPX(RobotMap.climberMotor3canID);
	
		reset();
	}	
	
	public void reset() {
		//climberMotor1.set(0.0);
		//climberMotor2.set(0.0);
		//climberMotor3.set(0.0);
	}

	public void stop () {
		//climberMotor1.stopMotor();
		//climberMotor2.stopMotor();
		//climberMotor3.stopMotor();
	}
	


	public void winchIn() {
		//climberMotor2.set(winchInSpeed);
		//climberMotor3.set(winchInSpeed);
	}
	public void winchOut() {
		//climberMotor2.set(winchOutSpeed);
		//climberMotor3.set(winchOutSpeed);
	}

	public void setWinchSpeed(double winchSpeed1, double winchSpeed2) {
		//climberMotor2.set(winchSpeed1);
		//climberMotor3.set(winchSpeed2);
	}
	
	public void setClimberArmMotorSpeed(double armSpeed) {
		//climberMotor1.set(armSpeed);

	}
	public double getRotateSpeed() {
		return 0; //climberMotor1.get();
	}
	public double getWinchSpeed() {
		return 0; //climberMotor2.get();
	}
	
	/**
	 * 
	public void rotateUp() {
		climberMotor1.set(rotateUpSpeed);
	}
	public void rotateDown() {
		climberMotor1.set(rotateDownSpeed);
	}
	 */
}
