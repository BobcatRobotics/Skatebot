package org.usfirst.frc.team177.robot;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team177.commands.DriveWithJoysticks;
import org.usfirst.frc.team177.lib.RioLogger;
import org.usfirst.frc.team177.lib.SmartDash;
import org.usfirst.frc.team177.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 2018 Robot Code - Main Class
 */
public class Robot extends TimedRobot {

	AHRS ahrs;
	
	/* Controls */
	public static final OI Controls = new OI();

	/* Subsystems */
	public static ExampleSubsystem m_subsystem = new ExampleSubsystem();

	// First limit switch (bottom or top? TBD)
	// DigitalInput limitswitch1 = new DigitalInput(9);

	/* Commands */
	// AutoCommand auto;
	DriveWithJoysticks driveJoy;
	// MoveElevator moveElevator;
	// MoveClimberArm moveClimberArm;

	/* SmartDashboard Information */
    private String gameData = "";
	private boolean gameDataFromField = false;
	
	
	// This boolean controls if the robot is in test recording or the robot
	// is running in competition mode
	boolean isCompetition = true;
	boolean processedGameInfo = false;
	boolean runSimpleAuto = false;
	int autoGameChecks;
	int autoGameCheckLimit = 250;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
        try {
         //ahrs = new AHRS(SerialPort.Port.kUSB1);
            //ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)200);
            ahrs = new AHRS(SPI.Port.kMXP);
            //ahrs = new AHRS(I2C.Port.kMXP);
        	ahrs.enableLogging(true);
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
		
		CameraServer.getInstance().startAutomaticCapture();
  	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		//Clear out the scheduler for testing, since we may have been in teleop before
		//we came int autoInit() change for real use in competition
		Scheduler.getInstance().removeAll();
	}

	@Override
	public void disabledPeriodic() {
		SmartDash.displayControlValues();
		displayGyroData();
	}

	/**
	 * Determine which side of the switches and scales is our color
	 * Drive there and drop off a cube
	 */
	@Override
	public void autonomousInit() {
		//Clear out the scheduler for testing, since we may have been in teleop before
		//we came int autoInit() change for real use in competition
		Scheduler.getInstance().removeAll();
		
		// Get initial Yaw when Auto mode initializes
		OI.gyro.zeroYaw();
		OI.AutoInitYawValue = OI.gyro.getYaw();

		// Reset Drive Train
		OI.driveTrain.reset();
		
		SmartDash.displayControlValues();

		// Beginning of a match, clear flag that says we have received game data
		// from the field, until we actually read a good game data message in this match
		gameDataFromField = false;
		processedGameInfo = false;
		runSimpleAuto = false;
		autoGameChecks = 0;

		// Get Game Data from field, Driver Station or default to no game data
		gameData = getGameData();

        if (gameDataFromField) {
        	if (isCompetition) {
				processedGameInfo = true;
			} 
        }
	}


	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		displayGyroData();
	    SmartDash.displayControlValues();
		
		if ((autoGameChecks > autoGameCheckLimit) && !runSimpleAuto && !processedGameInfo) {
			// we checked as much as we can, time to just try and cross the line
			RioLogger.errorLog("no good data, setting runSimpleAuto to true!");
			runSimpleAuto = true;
		} else {
			if (!processedGameInfo) {
		    	autoGameChecks++;
			}
		}
	}




	@Override
	public void teleopInit() {		
		//Clear out the scheduler for testing, since we may have been in teleop before
		//we came int autoInit() change for real use in competition
		Scheduler.getInstance().removeAll();		
	
		driveJoy = new DriveWithJoysticks();
		driveJoy.start();
		// moveElevator = new MoveElevatorWithJoystick();
		// moveElevator.start();
		// moveClimberArm = new MoveClimberArm();
		// moveClimberArm.start();
		
		ahrs.zeroYaw();

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		SmartDash.displayControlValues();
		displayGyroData();
	}			


	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		SmartDash.displayControlValues();
		displayGyroData();
	}
	

	
	// This method will attempt to get the game data from the field. If it is
	// invalid or cannot be retrieved then set a flag 
	private String getGameData() {
		final int MAX_GAMEDATA_LOOPS = 10;
		final double DELAY_FOR_GAMEDATA = 0.001;
		String gameData = "";

		// Read game data from driver station
		for (int i = 0; i < MAX_GAMEDATA_LOOPS; i++) {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
			if (gameData != null && !gameData.isEmpty()) {
				gameDataFromField = true;
				break;
			}
			Timer.delay(DELAY_FOR_GAMEDATA);
		}
		RioLogger.debugLog("Robot.getGameData() retrieved - " + gameData);
		RioLogger.debugLog("Robot.getGameData() gameDataFromField - " + gameDataFromField);
		System.out.println("gamedata from driver station = " + gameData);
		return gameData;
	}
	

	private void  displayGyroData () {
		/* Display 6-axis Processed Angle Data                                      */
		SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
		SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
		SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
		//SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
		//SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

		/* Display tilt-corrected, Magnetometer-based heading (requires             */
		/* magnetometer calibration to be useful)                                   */

		SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());

		/* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
		SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

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
