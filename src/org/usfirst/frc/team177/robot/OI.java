/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.usfirst.frc.team177.robot.commands.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	/* Joysticks */
	public static Joystick rightStick = new Joystick(RobotMap.rightJoystick);
	public static Joystick leftStick = new Joystick(RobotMap.leftJoystick);
	public static Joystick gamePad = new Joystick(RobotMap.gamePad);

	/* Buttons */
	public static Button btnSetMode0 = new JoystickButton(gamePad, RobotMap.gamePadMode0);
	public static Button btnSetMode1 = new JoystickButton(gamePad, RobotMap.gamePadMode1);
	public static Button btnSetMode2 = new JoystickButton(gamePad, RobotMap.gamePadMode2);
	public static Button btnSetMode3 = new JoystickButton(gamePad, RobotMap.gamePadMode3);
	public static Button btnSetMode4 = new JoystickButton(gamePad, RobotMap.gamePadMode4);

	static {
		btnSetMode0.whenPressed(new SetMode0());
		btnSetMode1.whenPressed(new SetMode1());
		btnSetMode2.whenPressed(new SetMode2());
		btnSetMode3.whenPressed(new SetMode3());
		btnSetMode4.whenPressed(new SetMode4());
	}
}
