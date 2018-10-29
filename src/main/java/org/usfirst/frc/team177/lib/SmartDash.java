package org.usfirst.frc.team177.lib;

import org.usfirst.frc.team177.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is used to centralize SmartDashboard operation (get and set)
 * @author bobcat177
 */
public class SmartDash {
	//public static PowerDistributionPanel pdp = new PowerDistributionPanel(); //Makes the PDP panel read 
	public static int currentSelection;
	
	// This method is used to display motors/controllers/switches
	public static void displayControlValues() {
        SmartDashboard.putNumber("Right Encoder Distance:", OI.driveTrain.getRightDistance());
        SmartDashboard.putNumber("Left Encoder Distance:", OI.driveTrain.getLeftDistance());
        SmartDashboard.putNumber("Right Encoder Rate:", OI.driveTrain.getRightRate());
        SmartDashboard.putNumber("Left Encoder Rate:", OI.driveTrain.getLeftRate());
        SmartDashboard.putNumber("Left Power Request:", OI.driveTrain.getLeftPower());
        SmartDashboard.putNumber("Left Percent applied:", OI.driveTrain.getLeftMotorPercent());
        SmartDashboard.putNumber("Right Power Request:", OI.driveTrain.getRightPower());
        SmartDashboard.putNumber("Right Percent applied:", OI.driveTrain.getRightMotorPercent());
        SmartDashboard.putNumber("Left Current applied:", OI.driveTrain.getLeftMotorCurrent());
        //SmartDashboard.putNumber("Auto Initial Gyro Value:", OI.AutoInitYawValue);
        //SmartDashboard.putNumber("Current Gyro Value:", OI.gyro.getYaw());
         
        //boolean shifterSolenoidState = OI.shifter.get();
        //SmartDashboard.putString("Shifter solenoid is: ", (shifterSolenoidState ? "ON" :"OFF"));
           
       	boolean shifterSwitchState = OI.trigShifter.get();
       	SmartDashboard.putString("Shifter switch is: ",(shifterSwitchState ?  "ON" : "OFF"));
       	
       	// SmartDashboard.putBoolean("Current Four Bar up/down state:",  FourBarUpDown.state);
       	
       	// OI.elevator.displayDashboard();
       	
       	// Real Time Logger
       	SmartDashboard.putData("RealTimeLog",OI.smartLog);
       	
      	/** Uncomment to test on Skateboard for lag 
       	SmartDashboard.putNumber("Current current draw of PDP channel 9: ", pdp.getCurrent(9));
       	SmartDashboard.putNumber("Current current draw of PDP channel 10: ", pdp.getCurrent(10)); 
       	SmartDashboard.putNumber("Total current draw on PDP: ", pdp.getTotalCurrent());
       	SmartDashboard.putNumber("Current Temperature of PDP in celcius: ", pdp.getTemperature());
       	SmartDashboard.putNumber("Current current draw of selected PDP channel: ", pdp.getCurrent(currentSelect()));
       	SmartDashboard.putNumber("Current PDP channel on current monitor", (double)currentSelect());
       	*/
	}
	
	
	public static void displayGameData(String gameData) {
		SmartDashboard.putString("Platform Data:", gameData);
	}
	

}
