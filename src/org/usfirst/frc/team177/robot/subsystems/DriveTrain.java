package org.usfirst.frc.team177.robot.subsystems;

import org.usfirst.frc.team177.robot.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrain extends Subsystem {
	/** Inverts drive direction **/
	private static final double INVERT_MOTOR = -1.0;
	
	private WPI_TalonSRX leftFront;
	private WPI_VictorSPX rightFront;
	private WPI_TalonSRX skateBotEncoder;
	private double leftPower = 0.0;
	private double rightPower = 0.0;
	private boolean invertLeft = false;
	private boolean invertRight = false;
	private int mode=1; // 0=raw, 1=velocity, 2=position
	private double modetime;
	private double mode2dt=0.02;
	private double mode3dist=8.0;
	private double modeleftPower = 0.0;
	private double moderightPower = 0.0;
	private double runtime = 1.5;
	private double ramptime = 0.5;
	private double tgtpwr = 0.25;
	private double ld = 0;     // left distance in inches
	private double rd = 0;     // right distance in inches
	private double ldlpv = 0;  // last pass value of ld
	private double rdlpv = 0;  // last pass value of rd
	private double dld = 0;    // change in left distance between now and last pass
	private double drd = 0;    // change in right distance between now and last pass
	private double ddist = 0;  // change in distance between now and last pass
	private double dist = 0;   // total absolut distance the robot has gone
	private double now = 0;    // time now in seconds
	private double nowlpv = 0; // last pass value of now
	private double dt = 20.0;  // delta time in seconds
	private double theta = 0.0; // angle of bot in degrees from -180 to +180 (ccw rotation is positive)
	private double thetalpv = 0.0; // angle of bot last time
	private double thetaavg = 0.0; // average of current and lpv value of angle
	private double dx = 0.0;   // change in x (delta x) between this pass and last pass
	private double dy = 0.0;   // change in y (delta y) between this pass and last pass
	private double y = 0.0;    // Y position of bot in inches
	private double x = 0.0;    // X position of bot in inches
	private double xlpv = 0.0; // last pass value of x
	private double ylpv = 0.0; // last pass value of y
	private double xt = 96.0;  // target x in inches
	private double yt = 40.0;  // target y in inches
	private double xtr = 0.0;  // xt - x
	private double ytr = 0.0;  // yt - y
	private double xu = 0.0;   // x component of the robot's direction facing Unit Vector
	private double yu = 0.0;   // y component of the robot's direction facing unit vector
	private double distproj = 0.0; // Projected distance to target
	private double kdist = 1.0;    // distance to velocity request gain ips/inches
	private double velreq = 0.0;   // Requested velocity in ips to drive toward target
	private double velreqlpv = 0.0; // last pass value of requested velocity
	private double velreqmax = 90.0; // Max allowable requested velocity
	private double accreqmax = 70.0; // max allowable change in requested velocity (max acceleration req)
	private double thetatarg = 0.0;  // angle of target from bots center (rather than field center)
	private double thetaturn = 0.0;  // angle to turn to target from robots current theta.
	private double disttol = 0.3;    // tolerance on meeting the projected distance.
	private double turnreq = 0.0;    // modifier to velocity to request turning
	                                 // positive turn req is turning ccw, so turnreq is added to right vel
	                                 //                        and subtracted from left vel requests.
	private double kturn = 0.2;     // propotionality constant to convert turnangle to velocity modifier
	
	
	private AHRS ahrs;

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithJoysticks());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }
    
    // Put methods for controlling and this subsystem by calling setters and getters
    // here. Call these from Commands.
	public void setupLeftMotor(int lf, boolean invert) {
		leftFront = new WPI_TalonSRX(lf);
		leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
		leftFront.setSelectedSensorPosition(0,0,0);
		leftFront.config_kF(0, 0.06, 0);
		leftFront.config_kP(0, 0.06, 0);
		leftFront.config_kI(0, 0.0006, 0);
		leftFront.config_kD(0, 2, 0);
		leftFront.config_IntegralZone(0, 2000, 0);
		invertLeft = invert;
		mode=1;
	}
	
	public void setupRightMotor(int rf, boolean invert) {
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
		invertRight = invert;
		mode=1;
	}

    // methods to switch the drivetrain control mode
	public void setmode0() {
		leftFront.config_kF(0, 0.0, 0);
		leftFront.config_kP(0, 0.0, 0);
		leftFront.config_kI(0, 0.0, 0);
		leftFront.config_kD(0, 0.0, 0);
		leftFront.config_IntegralZone(0, 0, 0);
		skateBotEncoder.config_kF(0, 0.0, 0);
		skateBotEncoder.config_kP(0, 0.0, 0);
		skateBotEncoder.config_kI(0, 0.0, 0);
		skateBotEncoder.config_kD(0, 0.0, 0);
		skateBotEncoder.config_IntegralZone(0, 0, 0);
		mode=0;
		modetime=0.0;
		ahrs.zeroYaw();
		ld = 0;
		rd = 0;
		ldlpv = 0;
		rdlpv = 0;
		dld = 0;
		drd = 0;
		ddist = 0;
		dist = 0;
		theta = 0.0;
		thetalpv = 0.0;
		thetaavg = 0.0;
		dx = 0.0;
		dy = 0.0;
		y = 0.0;
		x = 0.0;
		xlpv = 0.0;
		ylpv = 0.0;
		velreq=0.0;
		velreqlpv = 0.0;
	}
	
	public void setmode1() {
		leftFront.config_kF(0, 0.06, 0);
		leftFront.config_kP(0, 0.06, 0);
		leftFront.config_kI(0, 0.0006, 0);
		leftFront.config_kD(0, 2.0, 0);
		leftFront.config_IntegralZone(0, 2000, 0);
		//leftFront.configAllowableClosedloopError(0,40,0);
		skateBotEncoder.config_kF(0, 0.039, 0);
		skateBotEncoder.config_kP(0, 0.02, 0);
		skateBotEncoder.config_kI(0, 0.0003, 0);
		skateBotEncoder.config_kD(0, 1.5, 0);
		skateBotEncoder.config_IntegralZone(0, 4000, 0);
		//leftFront.configAllowableClosedloopError(0,40,0);
		// Clear any integral error from other modes
		leftFront.setIntegralAccumulator(0, 0, 0);
		skateBotEncoder.setIntegralAccumulator(0, 0, 0);
		mode=1;
		ahrs.zeroYaw();
		ld = 0;
		rd = 0;
		ldlpv = 0;
		rdlpv = 0;
		dld = 0;
		drd = 0;
		ddist = 0;
		dist = 0;
		theta = 0.0;
		thetalpv = 0.0;
		thetaavg = 0.0;
		dx = 0.0;
		dy = 0.0;
		y = 0.0;
		x = 0.0;
		xlpv = 0.0;
		ylpv = 0.0;
		velreq=0.0;
		velreqlpv = 0.0;
	}
	
	public void setmode2() {
		leftFront.config_kF(0, 0.0, 0);
		leftFront.config_kP(0, 0.6, 0);
		leftFront.config_kI(0, 0.00, 0);
		leftFront.config_kD(0, 0.0, 0);
		leftFront.config_IntegralZone(0, 2000, 0);
		skateBotEncoder.config_kF(0, 0.0, 0);
		skateBotEncoder.config_kP(0, 0.2, 0);
		skateBotEncoder.config_kI(0, 0.00, 0);
		skateBotEncoder.config_kD(0, 0.0, 0);
		skateBotEncoder.config_IntegralZone(0, 4000, 0);
		// Reset position sensor to zero (so we don't try to unwind position from other modes)
		leftFront.setSelectedSensorPosition(0,0,0);
		skateBotEncoder.setSelectedSensorPosition(0,0,0);
		// Clear any integral error from other modes
		leftFront.setIntegralAccumulator(0, 0, 0);
		skateBotEncoder.setIntegralAccumulator(0, 0, 0);
		mode=2;
		modetime=0.0;
		ahrs.zeroYaw();
		ld = 0;
		rd = 0;
		ldlpv = 0;
		rdlpv = 0;
		dld = 0;
		drd = 0;
		ddist = 0;
		dist = 0;
		theta = 0.0;
		thetalpv = 0.0;
		thetaavg = 0.0;
		dx = 0.0;
		dy = 0.0;
		y = 0.0;
		x = 0.0;
		xlpv = 0.0;
		ylpv = 0.0;
		velreq=0.0;
		velreqlpv = 0.0;
	}

	public void setmode3() {
		leftFront.config_kF(0, 0.078, 0);
		leftFront.config_kP(0, 0.4, 0);
		leftFront.config_kI(0, 0.000, 0);
		leftFront.config_kD(0, 0.0, 0);
		leftFront.config_IntegralZone(0, 3072, 0);
		skateBotEncoder.config_kF(0, 0.047, 0);
		skateBotEncoder.config_kP(0, 0.12, 0);
		skateBotEncoder.config_kI(0, 0.000, 0);
		skateBotEncoder.config_kD(0, 0.0, 0);
		skateBotEncoder.config_IntegralZone(0, 5120, 0);
		// Reset position sensor to zero (so we don't try to unwind position from other modes)
		leftFront.setSelectedSensorPosition(0,0,0);
		skateBotEncoder.setSelectedSensorPosition(0,0,0);
		// Clear any integral error from other modes
		leftFront.setIntegralAccumulator(0, 0, 0);
		skateBotEncoder.setIntegralAccumulator(0, 0, 0);
		// Set cruise velocity and acceleration
		leftFront.configMotionAcceleration(4915, 0);
		leftFront.configMotionCruiseVelocity(4915, 0);
		skateBotEncoder.configMotionAcceleration(8192, 0);
		skateBotEncoder.configMotionCruiseVelocity(8192, 0);
		
		mode=3;
		modetime=0.0;
		ahrs.zeroYaw();
		ld = 0;
		rd = 0;
		ldlpv = 0;
		rdlpv = 0;
		dld = 0;
		drd = 0;
		ddist = 0;
		dist = 0;
		theta = 0.0;
		thetalpv = 0.0;
		thetaavg = 0.0;
		dx = 0.0;
		dy = 0.0;
		y = 0.0;
		x = 0.0;
		xlpv = 0.0;
		ylpv = 0.0;
		velreq=0.0;
		velreqlpv = 0.0;
	}

	public void setmode4() {
		leftFront.config_kF(0, 0.067, 0);
		leftFront.config_kP(0, 0.02, 0);
		leftFront.config_kI(0, 0.0002, 0);
		leftFront.config_kD(0, 0.0, 0);
		leftFront.config_IntegralZone(0, 1000, 0);
		leftFront.configAllowableClosedloopError(0,0,0);

		skateBotEncoder.config_kF(0, 0.040, 0);
		skateBotEncoder.config_kP(0, 0.01, 0);
		skateBotEncoder.config_kI(0, 0.0001, 0);
		skateBotEncoder.config_kD(0, 0.0, 0);
		skateBotEncoder.config_IntegralZone(0, 2000, 0);
		skateBotEncoder.configAllowableClosedloopError(0,0,0);
		
		// Clear any integral error from other modes
		leftFront.setIntegralAccumulator(0, 0, 0);
		skateBotEncoder.setIntegralAccumulator(0, 0, 0);
		mode=4;
		ahrs.zeroYaw();
		ld = 0;
		rd = 0;
		ldlpv = 0;
		rdlpv = 0;
		dld = 0;
		drd = 0;
		ddist = 0;
		dist = 0;
		theta = 0.0;
		thetalpv = 0.0;
		thetaavg = 0.0;
		dx = 0.0;
		dy = 0.0;
		y = 0.0;
		x = 0.0;
		xlpv = 0.0;
		ylpv = 0.0;
		velreq=0.0;
		velreqlpv = 0.0;
	}

	public void setupGyro() {
		/***********************************************************************
		 * navX-MXP:
		 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
		 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
		 * Multiple navX-model devices on a single robot are supported.
		 ************************************************************************/
        ahrs = new AHRS(SPI.Port.kMXP);
    	ahrs.enableLogging(false);
	}
	
	public void zeroGyroYaw() {
		ahrs.zeroYaw();
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

	public double getRightMotorCurrent() {
		return rightFront.getOutputCurrent();
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
		if ((leftPwr < 0.02) && (leftPwr > -0.02)) {
			leftPwr=0.0;
			//leftFront.setIntegralAccumulator(0, 0, 0);
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
		if ((rightPwr < 0.02) && (rightPwr > -0.02)) {
			rightPwr=0.0;
			//skateBotEncoder.setIntegralAccumulator(0, 0, 0);
		}
	
		this.rightPower = rightPwr;
	}

	public void drive() {
		drive(leftPower,rightPower);
	}
	
	public void drive(double leftPwr, double rightPwr) {
		if (invertLeft) leftPwr *= INVERT_MOTOR;
		if (invertRight) rightPwr *= INVERT_MOTOR;

		// how much time does the robot think has elapsed:
		nowlpv = now;  // Save the last pass value of now
		now = Timer.getFPGATimestamp();  // What time is it now
		dt = now - nowlpv;		// delta time is the differernce between now and now last time

		// Save old value of ld and rd
		ldlpv = ld;
		rdlpv = rd;
		
		// Calculate the left and right distance at this moment in time:
		//
		// First the left side -- 
		// left distance is negative on the skatebot
		// There are 4096 tics per sensor rotation and 3 sensor rotation per wheel rotation
		// and the wheels are 3 13/16 inches in diameter (they were 3 7/8 new)
		// so, take the position in native units and:
		ld=-1.0*leftFront.getSelectedSensorPosition(0)*0.33333333*0.000244141*3.141592*3.8125;
		
		// right distance is positive on the skatebot
		// There are 4096 tics per sensor rotation and 5 sensor rotation per wheel rotation
		// and the wheels are 3 13/16 inches in diameter (they were 3 7/8 new)
		// so, take the position in native units and:
		rd=skateBotEncoder.getSelectedSensorPosition(0)*0.2*0.000244141*3.141592*3.8125;
		
		// Change in left and right sides are:
		dld = ld - ldlpv;
		drd = rd - rdlpv;
		
		// Then the distance of the skatebot moved since last time is the average delta dist on left and right:
		ddist=(dld+drd)/2.0;
		dist = dist + ddist;

		// What is the angle we make to the X axis (make counter-clockwise rotation positive)
		thetalpv = theta;
		theta = -1.0*ahrs.getYaw();
		thetaavg = (theta+thetalpv)/2.0;
		
		// Now calculate the x and y of the drivetrain.
		//  first the incremental change in x and y:
		dx = ddist*Math.cos(Math.toRadians(thetaavg));
		dy = ddist*Math.sin(Math.toRadians(thetaavg));		
		//  then add change in x and y to last pass value of total x and y: 
		xlpv = x;
		ylpv = y;
		x=xlpv+dx;
		y=ylpv+dy;

		// Calculate X & Y  between target and bot:
		xtr = xt - x;
		ytr = yt - y;
		
		// Calculate X and Y of a unit vector facing forward out of the center of the bot:
		xu = Math.cos(Math.toRadians(theta));
		yu = Math.sin(Math.toRadians(theta));

		// Calculate distance from robot to target that projected in the robots current direction
		distproj = xtr * xu + ytr * yu;
		
		if (distproj < disttol) distproj = 0.0;
		
		// Set a velocity request proportional to the projected distance
		velreqlpv = velreq;
		velreq = kdist * distproj;
		// Limit the velocity request to a max value and rate limit the change
		if (velreq > velreqmax) velreq = velreqmax;
		if (velreq < -1.0*velreqmax) velreq = -1.0*velreqmax;
		if (velreq > (velreqlpv + accreqmax * dt) ) velreq = velreqlpv + accreqmax *dt;
		if (velreq < (velreqlpv - accreqmax * dt) ) velreq = velreqlpv - accreqmax *dt;
 		
		// Calc theta of target from bot center  TODO: maybe move above velreq, and calc velreq left/right here? maybe increase izone? look at how it works for velocity mode talon PID
		thetatarg = Math.toDegrees(Math.atan2(ytr, xtr));
		thetaturn = thetatarg - theta;
		if (thetaturn > 180.0) thetaturn = thetaturn - 360.0;
		if (thetaturn < -180.0) thetaturn = thetaturn + 360.0;
		turnreq = kturn * thetaturn;
		if (thetaturn < 1.0) turnreq = 0.0;
		
		
		if (mode == 0) {
			runtime = 3.0;
			ramptime = 0.2;
			tgtpwr = 0.5 ;
		/*
			// Just send basic request from sticks
			modetime = modetime + mode2dt; // assume 20 msec per pass for now.
			// Start setting power to the value of the ramp:
			leftPwr = -1.0 * modetime/ramptime * tgtpwr;
			rightPwr = modetime/ramptime * tgtpwr;
			// Unless time is above the ramptime, then just stick to the target power:
			if (modetime > ramptime) {
				leftPwr=-1.0*tgtpwr;
				rightPwr=tgtpwr;
			}
			// Unless time is above the total runtime, then just set power to zero:
			if (modetime > runtime) {
				leftPwr=0.0;
				rightPwr=0.0;
			} 
			this.leftPower = leftPwr;
			this.rightPower = rightPwr;
		*/
			
			leftFront.set(leftPwr);
			skateBotEncoder.set(rightPwr);
		}
		if (mode == 1) {
			// Use request from sticks to set a velocity target
			// 8000 and 13329 are arbitrary limits on +/- velocity in native units/100msec
			// they are different because left side has 3 sensor rotation per 1 wheel rotation
			// and right side has 5 sensor rotations per 1 wheel rotation
			leftFront.set(ControlMode.Velocity, leftPwr*8000.0);
	        //use skateBotEncoder.set(...)  -- note, that this works because rightfront
	        //                                 has been set to follow skateBotEncoder.
			skateBotEncoder.set(ControlMode.Velocity, rightPwr*13329.0);
		}
		if (mode == 2) {
			// Drive to a position target, stomp on power request from sticks.
			// try calculating a curve of position from 0 to 8 rotations (each rotation is about a 12 inches)
            // calc right power first, since forward on skatebot is positive power on the right
			rightPwr=0.032814 - 0.427667*modetime + 3.07246*modetime*modetime - 0.6827695*modetime*modetime*modetime;
			leftPwr=-1.0*rightPwr; // Drive straight
			//leftPwr=rightPwr;    // Spin in a circle
			modetime = modetime + mode2dt; // assume 20 msec per pass for now.
			// Make time count up & down from 0.0 to 3.0
			if (modetime > 3.0) {
				modetime = 3.0;
				mode2dt = -1.0*mode2dt;
			}
			if (modetime < 0.0) {
				modetime = 0.0;
				mode2dt = -1.0*mode2dt;
			}

			// try rotations (where each rotation is 4096 units, and left side
			//                       gearing is such that 3 sensor rotations=1wheel rotation)
	        leftFront.set(ControlMode.Position, leftPwr*4096.0*3.0);
			// try rotations (where each rotation is 4096 units, and right side
			//                       gearing is such that 5 sensor rotations=1wheel rotation)
	        skateBotEncoder.set(ControlMode.Position, rightPwr*4096.0*5.0);		
		}
		if (mode == 3) {
			// Drive to a position target, stomp on power request from sticks.

			// Leave this modetime stuff here for now, may use later
			modetime = modetime + mode2dt; // assume 20 msec per pass for now.
			// Make time count up & down from 0.0 to 3.0
			if (modetime > 3.0) {
				modetime = 3.0;
				mode3dist = 0.0;
				mode2dt = -1.0*mode2dt;
			}
			if (modetime < 0.0) {
				modetime = 0.0;
				mode3dist = 8.0;
				mode2dt = -1.0*mode2dt;
			}

			// try -8.0 rotations (where each rotation is 4096 units, and left side
			//                       gearing is such that 3 sensor rotations=1wheel rotation)
	        leftFront.set(ControlMode.MotionMagic, -1.0*mode3dist*4096.0*3.0);
			// try 8.0 rotations (where each rotation is 4096 units, and right side
			//                       gearing is such that 5 sensor rotations=1wheel rotation)
	        skateBotEncoder.set(ControlMode.MotionMagic, mode3dist*4096.0*5.0);		
		}
		if (mode == 4) {
//			modetime = modetime + mode2dt; // assume 20 msec per pass for now.
//			// Make time count up & down from 0.0 to 2.0
//			if (modetime > 2.0) {
//				modetime = 2.0;
//				mode2dt = -1.0*mode2dt;
//				moderightPower=0.5;
//			}
//			if (modetime < 0.0) {
//				modetime = 0.0;
//				mode2dt = -1.0*mode2dt;
//				moderightPower=0.25;
//			}
//			leftPwr=modeleftPower;
//			rightPwr=moderightPower;
//			// Use request from sticks to set a velocity target
//			// 8000 and 13329 are arbitrary limits on +/- velocity in native units/100msec
//			// they are different because left side has 3 sensor rotation per 1 wheel rotation
//			// and right side has 5 sensor rotations per 1 wheel rotation
//			leftFront.set(ControlMode.Velocity, leftPwr*8000.0);
//	        //use skateBotEncoder.set(...)  -- note, that this works because rightfront
//	        //                                 has been set to follow skateBotEncoder.
//			skateBotEncoder.set(ControlMode.Velocity, rightPwr*13329.0);
			
			// Use the velocity request to set a velocity target for the left and right sides:

			// There are 4096 tics per sensor rotation and 3 or 5 sensor rotation per wheel rotation
			// and the wheels are 3 13/16 inches in diameter (they were 3 7/8 new),
			// and the velocity request should be in native units per 100 msec:
            // 3.0*4096.0*0.1/(3.141592*3.8125) = 102.594 (negative as left side spins backward)
			// 5.0*4096.0*0.1/(3.141592*3.8125) = 170.9898
			leftFront.set(ControlMode.Velocity, -102.594*(velreq-turnreq));
			skateBotEncoder.set(ControlMode.Velocity, 170.9898*(velreq+turnreq));
		}

	}

	public void stop() {
		leftPower = 0.0;
		rightPower = 0.0;
		leftFront.set(0.0);
		skateBotEncoder.set(0.0);
		rightFront.set(0.0);
	}

	public void reset() {
		leftPower = 0.0;
		rightPower = 0.0;
		rightFront.follow(skateBotEncoder);
	}
	
	public void displayDriveTrainData() {
		SmartDashboard.putNumber("Mode", mode);
		SmartDashboard.putNumber("modetime", modetime);
		SmartDashboard.putNumber("Left Encoder Distance",    getLeftDistance());
		SmartDashboard.putNumber("Left Encoder Velocity",    getLeftRate());
		SmartDashboard.putNumber("Left Motor power request", getLeftPower());
		SmartDashboard.putNumber("Left Motor output %",      getLeftMotorPercent());
		SmartDashboard.putNumber("Left Motor output amps",   getLeftMotorCurrent());
		SmartDashboard.putBoolean("Left power request inverted",   invertLeft);
		
		SmartDashboard.putNumber("Right Encoder Distance",    getRightDistance());
		SmartDashboard.putNumber("Right Encoder Velocity",    getRightRate());
		SmartDashboard.putNumber("Right Motor power request", getRightPower());
		SmartDashboard.putNumber("Right Motor output %",      getRightMotorPercent());
		SmartDashboard.putNumber("Right Motor output amps",   getRightMotorCurrent());
		SmartDashboard.putBoolean("Right power request inverted",   invertRight);
		
		SmartDashboard.putNumber("now", now);
		SmartDashboard.putNumber("dt", dt);
		SmartDashboard.putNumber("ld", ld);
		SmartDashboard.putNumber("rd", rd);
		SmartDashboard.putNumber("dld", dld);
		SmartDashboard.putNumber("drd", drd);
		SmartDashboard.putNumber("dist", dist);
		SmartDashboard.putNumber("ddist", ddist);
		SmartDashboard.putNumber("dx", dx);
		SmartDashboard.putNumber("dy", dy);
		SmartDashboard.putNumber("x", x);
		SmartDashboard.putNumber("y", y);
		SmartDashboard.putNumber("velreq", velreq);
		SmartDashboard.putNumber("distproj", distproj);
		SmartDashboard.putNumber("xt", xt);
		SmartDashboard.putNumber("yt", yt);
		SmartDashboard.putNumber("xtr", xtr);
		SmartDashboard.putNumber("ytr", ytr);
		SmartDashboard.putNumber("thetatarg", thetatarg);
		SmartDashboard.putNumber("thetaturn", thetaturn);
		SmartDashboard.putNumber("turnreq", turnreq);
		
	}
	
	public void displayGyroData () {
		/* Display 6-axis Processed Angle Data                                      */
		SmartDashboard.putBoolean("IMU_Connected",        ahrs.isConnected());
		SmartDashboard.putBoolean("IMU_IsCalibrating",    ahrs.isCalibrating());
		SmartDashboard.putNumber( "IMU_Yaw",              ahrs.getYaw());
		SmartDashboard.putNumber( "IMU_Pitch",            ahrs.getPitch());
		SmartDashboard.putNumber( "IMU_Roll",             ahrs.getRoll());

		/* Display tilt-corrected, Magnetometer-based heading (requires             */
		/* magnetometer calibration to be useful)                                   */
		//SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());

		/* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
		//SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

		/* These functions are compatible w/the WPI Gyro Class, providing a simple  */
		/* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
		SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
		//SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

		/* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
		//SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
		//SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
		//SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
		//SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

		/* Display estimates of velocity/displacement.  Note that these values are  */
		/* not expected to be accurate enough for estimating robot position on a    */
		/* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
		/* of these errors due to single (velocity) integration and especially      */
		/* double (displacement) integration.                                       */
		//SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
		//SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
		//SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
		//SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());

		/* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
		/* NOTE:  These values are not normally necessary, but are made available   */
		/* for advanced users.  Before using this data, please consider whether     */
		/* the processed data (see above) will suit your needs.                     */
		//SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
		//SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
		//SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
		//SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
		//SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
		//SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
		//SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
		//SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
		//SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
		//SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
		//SmartDashboard.putNumber(   "IMU_Timestamp",        ahrs.getLastSensorTimestamp());

		/* Omnimount Yaw Axis Information                                           */
		/* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
		//AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
		//SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
		//SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );

		/* Sensor Board Information                                                 */
		//SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());

		/* Quaternion Data                                                          */
		/* Quaternions are fascinating, and are the most compact representation of  */
		/* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
		/* from the Quaternions.  If interested in motion processing, knowledge of  */
		/* Quaternions is highly recommended.                                       */
		//SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
		//SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
		//SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
		//SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());

		/* Connectivity Debugging Support                                           */
		//SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
		//SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
	}
}