/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team177.robot;

import org.usfirst.frc.team177.lib.RioLogger;
import org.usfirst.frc.team177.lib.RioLoggerThread;
import org.usfirst.frc.team177.lib.SmartDashLog;
import org.usfirst.frc.team177.subsystems.DriveTrain;
import org.usfirst.frc.team177.subsystems.NavxGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	/* Loggers */
	public static RioLoggerThread logFile;
	public static SmartDashLog smartLog = new SmartDashLog();

	/* Drive Chain */
	public static DriveTrain driveTrain = new DriveTrain();


	/* Motors */
	// Competition bot has VictorSPXs for the cube arm motors
	//public static WPI_VictorSPX cubeLeftMotor = new WPI_VictorSPX(RobotMap.cubePickupLeft);
	//public static WPI_VictorSPX cubeRightMotor = new WPI_VictorSPX(RobotMap.cubePickupRight);

	// Practice bot has TalonSRXs for the cube arm motors (but CanIDs should be same)
	//public static WPI_TalonSRX cubeLeftMotor = new WPI_TalonSRX(RobotMap.cubePickupLeft);
	//public static WPI_TalonSRX cubeRightMotor = new WPI_TalonSRX(RobotMap.cubePickupRight);

	// public static TalonMagEncoder elevatorEncoder = new TalonMagEngcoder(3);

	/* Solenoids */
	//public static Solenoid cubeArms = new Solenoid(RobotMap.cubePickupSolenoid); /* Controls the Cube Arms */

	/* Gyro */
	public static NavxGyro gyro;
	public static double AutoInitYawValue;

	/* Joysticks */
	public static Joystick rightStick = new Joystick(RobotMap.rightJoystick);
	public static Joystick leftStick = new Joystick(RobotMap.leftJoystick);
	public static Joystick gamePad = new Joystick(RobotMap.gamePad);

	/* Buttons */
	// public static Button btnCubePickup = new JoystickButton(gamePad, RobotMap.gamePadCubePickup);

	/* Triggers */
	//public static Trigger trigShifter = new JoystickButton(rightStick, RobotMap.rightJoystickShifter);
	public static Trigger trigShifter = new JoystickButton(gamePad, RobotMap.rightJoystickShifter);

	/* DigitBoard */
	// public static DigitBoard digitBoard = DigitBoard.getInstance();

	static {

		driveTrain.setRightMotors(RobotMap.driveRightMotorFront, RobotMap.driveRightMotorMiddle,
				RobotMap.driveRightMotorRear);
		driveTrain.setLeftMotors(RobotMap.driveLeftMotorFront, RobotMap.driveLeftMotorMiddle,
				RobotMap.driveLeftMotorRear);

		driveTrain.setLeftMotorsReverse(false);
	
		// Start Logging Thread
		logFile = RioLoggerThread.getInstance();
		RioLogger.log("OI static block finished.");
	}
}
