package org.usfirst.frc.team177.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DriveTrain {
	/** Inverts drive direction **/
	private static final double INVERT_MOTOR = -1.0;
	
	private WPI_TalonSRX leftFront;
	//private WPI_VictorSPX leftMiddle;
	//private WPI_VictorSPX leftRear;
	private WPI_VictorSPX rightFront;
	//private WPI_VictorSPX rightMiddle;
	//private WPI_VictorSPX rightRear;
	private WPI_TalonSRX skateBotEncoder;
	
	//private GrayHill leftEncoder;
	//private GrayHill rightEncoder;
	
	private boolean invertLeft = true;
	
	private double leftPower = 0.0;
	private double rightPower = 0.0;
	
	
	public DriveTrain() {
		super();
	}
	
	public void setLeftMotors(int lf,int lm,int lr) {
		leftFront = new WPI_TalonSRX(lf);
		leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
		leftFront.setSelectedSensorPosition(0,0,0);
		leftFront.config_kF(0, 0.06, 0);
		leftFront.config_kP(0, 0.06, 0);
		leftFront.config_kI(0, 0.0006, 0);
		leftFront.config_kD(0, 2, 0);
		leftFront.config_IntegralZone(0, 2000, 0);
		
		//leftMiddle = new WPI_VictorSPX(lm);
		//leftRear = new WPI_VictorSPX(lr);
		
	}
	
	public void setRightMotors(int rf,int rm,int rr) {
		rightFront = new WPI_VictorSPX(rf);
		skateBotEncoder= new WPI_TalonSRX(2);
		rightFront.follow(skateBotEncoder);
		skateBotEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
		skateBotEncoder.setSelectedSensorPosition(0,0,0);
		skateBotEncoder.config_kF(0, 0.039, 0);
		skateBotEncoder.config_kP(0, 0.02, 0);
		skateBotEncoder.config_kI(0, 0.0003, 0);
		skateBotEncoder.config_kD(0, 1.5, 0);
		skateBotEncoder.config_IntegralZone(0, 4000, 0);

		//rightMiddle = new WPI_VictorSPX(rm);
		//rightRear = new WPI_VictorSPX(rr);
	}

	public void setLeftMotorsReverse(boolean invert) {
		invertLeft = invert;
	}
	
	public void setLeftEncoder(int leftEncCh1, int leftEncCh2) {
		//leftEncoder = new GrayHill(leftEncCh1, leftEncCh2, false);
	}

	public double getLeftDistance() {
		return leftFront.getSelectedSensorPosition(0);
	}

	public double getLeftRate() {
		return leftFront.getSelectedSensorVelocity(0);
	}
	
	public double getLeftMotorPercent() {
		return leftFront.getMotorOutputPercent();
	}
	
	public double getLeftMotorCurrent() {
		return leftFront.getOutputCurrent();
	}
	
	public double getRightMotorPercent() {
		return rightFront.getMotorOutputPercent();
	}
	
	public void setRightEncoder(int rightEncCh1, int rightEncCh2) {
		//rightEncoder = new GrayHill(rightEncCh1, rightEncCh2, true);
	}

	public double getRightDistance() {
		return skateBotEncoder.getSelectedSensorPosition(0);
	}

	public double getRightRate() {
		return skateBotEncoder.getSelectedSensorVelocity(0);
	}

	public double getLeftPower() {
		return leftPower;
	}

	public void setLeftPower(double leftPwr) {
		if (leftPwr > 1.0)
			leftPwr = 1.0;
		else
		if (leftPwr < -1.0)
			leftPwr = -1.0;
		if ((leftPwr < 0.05) && (leftPwr > -0.05)) {
			leftPwr=0.0;
			leftFront.setIntegralAccumulator(0, 0, 0);
		}
		
		this.leftPower = leftPwr;
	}

	public double getRightPower() {
		return rightPower;
	}

	public void setRightPower(double rightPwr) {
		if (rightPwr > 1.0)
			rightPwr = 1.0;
		else
		if (rightPwr < -1.0)
			rightPwr = -1.0;
		if ((rightPwr < 0.05) && (rightPwr > -0.05)) {
			rightPwr=0.0;
			skateBotEncoder.setIntegralAccumulator(0, 0, 0);
		}

		this.rightPower = rightPwr;
	}

	public void drive() {
		drive(leftPower,rightPower);
	}
	
	public void drive(double leftPwr, double rightPwr) {
		if (invertLeft )
			leftPwr *= INVERT_MOTOR;
		else
			rightPwr *= INVERT_MOTOR;
		
		//leftFront.set(leftPwr);
        leftFront.set(ControlMode.Velocity, leftPwr*8000.0);
		
		//leftMiddle.set(leftPwr);
		//leftRear.set(leftPwr);

        
        //rightFront.set(rightPwr);
        //skateBotEncoder.set(rightPwr);
        skateBotEncoder.set(ControlMode.Velocity, rightPwr*13329.0);
        
		//rightMiddle.set(rightPwr);
		//rightRear.set(rightPwr);
	}

	public void stop() {
		leftPower = 0.0;
		rightPower = 0.0;
		rightFront.follow(skateBotEncoder);
		
		// TODO :: Could also use .stopMotor()
		leftFront.set(0.0);
		//leftMiddle.set(0.0);
		//leftRear.set(0.0);
		rightFront.set(0.0);
		//rightMiddle.set(0.0);
		//rightRear.set(0.0);
	}

	public void reset() {
		leftPower = 0.0;
		rightPower = 0.0;
		rightFront.follow(skateBotEncoder);
		//leftEncoder.reset();
		//rightEncoder.reset();
	}
}