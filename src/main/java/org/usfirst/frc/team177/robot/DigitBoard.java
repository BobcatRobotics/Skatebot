package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * REV Digit MXP Display (am-3223)
 * About the button get functions: 
 * - get(DigiButton) will return when the button is pressed down
 * - getOnRisingEdge(DigiButton) will return true when the button is released
 * - getLatch(DigiButton) will toggle between true and false. 
 * About the potentiometer:
 * - getPotentiometer function returns the value of the potentiometer (range 0 - 10).
 */

public class DigitBoard {

	public static DigitBoard instance = new DigitBoard();
	private Thread boardThread;
	private boolean isRunning = false;

	private I2C displayBoard;
	private DigitalInput buttonA;
	private DigitalInput buttonB;
	private AnalogInput potentiometer;

	private byte[] displayData;

	private static final int A_MASK = 0b0000000000000001;
	private static final int B_MASK = 0b0000000000000010;
	private volatile int inputMask;
	private volatile int inputMaskRising;
	private volatile int inputMaskRisingLast;


	private static class DigitBoardThread implements Runnable {
		private DigitBoard digiBoard;

		DigitBoardThread(DigitBoard b) {
			digiBoard = b;
		}

		public void run() {
			digiBoard.updateDisplay();
		}
	}


	/**
	 * Returns the instance of the Digit Board
	 * @return DigitBoard
	 */
	public static DigitBoard getInstance() {
		return DigitBoard.instance;
	}

	private DigitBoard() {
		boardThread = new Thread(new DigitBoardThread(this), "MXP_Display_Board");
		boardThread.setPriority((Thread.NORM_PRIORITY + Thread.MAX_PRIORITY) / 2);
		DisplayInit();
		start();
	}

	// TODO:: Check the buttons and potentiometer are always at these locations
	// TODO:: XXXXXXXXXXX
	private void DisplayInit() {
		displayBoard = new I2C(I2C.Port.kMXP, 0x70);
		buttonA = new DigitalInput(9);
		buttonB = new DigitalInput(10);
		potentiometer = new AnalogInput(3);
		
		displayData = new byte[10];
		
	}
	
	public void start() {
		if (isRunning)
			return;
		isRunning = true;
		boardThread = new Thread(new DigitBoardThread(this));
		boardThread.start();
	}

	public void stop() {
		isRunning = false;
	}

	/**
	 * Called from the Threads run method
	 */
	private void updateDisplay() {
	
		displayBoard.writeBulk(DigiMap.OSC);
		displayBoard.writeBulk(DigiMap.BRIGHT);
		displayBoard.writeBulk(DigiMap.BLINK);

		while (isRunning) {
			update();
			displayBoard.writeBulk(displayData);
			Timer.delay(0.1);

		}
	}
	/**
	 * Function for updating buttons, and its helper functions. getA and getB will
	 * simply return if the button is pushed (the button being pushed returns false)
	 * getALatch and getBLatch will remain true until the data is read.
	 * getAonRisingEdge and getBOnRisingEdgeis true only when the value has gone
	 * from false to true (it will only be true when the button is released.
	 */
	public void update() {
		int current_mask = get_input_mask();

		inputMask |= current_mask;

		inputMaskRising |= (~inputMaskRisingLast & current_mask);
		inputMaskRisingLast = current_mask;
	}

	private int get_input_mask() {
		int mask = 0;
		mask |= (get(DigiButton.BUTTON_A) ? 1 : 0) << A_MASK;
		mask |= (get(DigiButton.BUTTON_B) ? 1 : 0) << B_MASK;
		return mask;
	}

	private boolean getRawButtonOnRisingEdge(int button_mask) {
		button_mask = button_mask << 1;
		// Compute a clearing mask for the button.
		int clear_mask = 0b1111111111111111 - button_mask;
		// Get the value of the button - 1 or 0
		boolean value = (inputMaskRising & button_mask) != 0;
		// Mask this and only this button back to 0
		inputMaskRising &= clear_mask;
		return value;
	}

	private boolean getRawButtonLatch(int button_mask) {
		// Compute a clearing mask for the button.
		int clear_mask = 0b1111111111111111 - button_mask;
		// Get the value of the button - 1 or 0
		boolean value = (inputMask & button_mask) != 0;
		// Mask this and only this button back to 0
		inputMask &= clear_mask;
		return value;
	}

	public boolean get(DigiButton btn) {
		if (DigiButton.BUTTON_A == btn)
			return (!buttonA.get());
		else
			return (!buttonB.get());
	}
	
	public boolean getLatch(DigiButton btn) {
		if (DigiButton.BUTTON_A == btn)
			return getRawButtonLatch(A_MASK);
		else
			return getRawButtonLatch(B_MASK);

	}
	
	public boolean getOnRisingEdge(DigiButton btn) {
		if (DigiButton.BUTTON_A == btn)
			return getRawButtonOnRisingEdge(A_MASK);
		else
			return getRawButtonOnRisingEdge(B_MASK);
	}

	/**
	 * Returns the current value of the potentiometer
	 * @return an integer between 0 - 10
	 */
	public int getPotentiometer() {
		double val = (double) potentiometer.getValue();// integer between 219 and 202 (check this every init)
		
		val -= 202.0;
		val /= 2.0;
		return (int)val;
	}

	public void clearDisplay( ) {
		for (int pos = 0; pos < 10; pos += 2 ) {
			displayData[pos] = DigiMap.BLANK[pos /2][0];
			displayData[pos+1] = DigiMap.BLANK[pos /2][1];
		
		}
	}
	
	/*
	 * Display a string (up to 4 characters)
	 */
	public void display(String value) {
		clearDisplay();
		for (int ctr = 0; ctr < value.length(); ctr++) {
			char letter = value.charAt(ctr);
			displayData[(3 - ctr) * 2 + 2] = DigiMap.getRawLower(letter);
			displayData[(3 - ctr) * 2 + 3] = DigiMap.getRawUpper(letter);
		}
	}

	/**
	 * display a number (up to 4 digits)
	 */
	public void display(double value) {
		display(String.format("%4.1f",value));
	}
	
	/**
	 * display a number (specific decimal digits)
	 */
	public void display(double value,int decimals) {
		int dec = (decimals > 3) ? 3 : decimals;
		String formatter = "%4." + dec + "f";
		display(String.format(formatter,value));
	}
	
	/**
	 * display an integer
	 */
	public void display(int value) {
		display(String.format("%4d",value));
	}

}
