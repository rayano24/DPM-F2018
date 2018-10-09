package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.utility.Delay;

public class LightLocalizer implements Runnable {
	
	
	private static EV3LargeRegulatedMotor leftMotor = Localization.leftMotor;
	private static EV3LargeRegulatedMotor rightMotor = Localization.rightMotor;
	
	private static final EV3ColorSensor colorSensor = Localization.colorSensor;
	
	protected static Odometer odometer = Localization.odometer;

	
	//private static final SensorMode redMode = colorSensor.getRedMode(); // measure intensity of reflected light
	//pprivate float[] colour = new float[redMode.sampleSize()];
	

	
	// distance between the color sensor and the center of rotation
	public static final double sensorOffset = 15.9;

	
	
	// begins light localization post Ultrasonic sensor localization i.e, it is oriented at 0 degrees
	public void run() {

		lightLocalization();
	}

	
	
	
	/**
	 * Completes light localization. Calls localizationForward. Attempt to orient robot at 0 degrees & (0,0).
	 */
	public void lightLocalization() {
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);


		
		// robot will go in the x direction
		localizationForward(1000, false, true);
		
		// go back to origin tile
		localizationForward(4000, false, false);

		// turns robot 90 degrees to the right
		Localization.turnTo((odometer.getXYT()[2] + 90) % 360);
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);

		// go in the y direction
		localizationForward(1000, true, true);
		// go back to origin tile
		localizationForward(4000, true, false);

		// Calling moveUntilLine has allowed us to reset the x-y coordinates,
		// therefore travelTo(0, 0) will go the 0,0 we actually want it to go to.
		
		// will travel to 0,0. delay added to  ensure accuracy 
		Localization.travelTo(0, 0);
		Localization.turnTo(0);
		
		leftMotor.setSpeed(30);
		rightMotor.setSpeed(30);
		
		leftMotor.forward();
		rightMotor.forward();
		
		Delay.msDelay(500);
		
		leftMotor.stop();
		rightMotor.stop();
		
		//done
	}

	
	/**
	 * 
	 * @param delay delay after line is detected
	 * @param pos True if robot is crossing the X axis
	 * @param goForward True if robot is going forward
	 */
	private void localizationForward(int delay, boolean crossX, boolean goForward) {
		if (goForward) {
			leftMotor.forward();
			rightMotor.forward();
		} else {
			leftMotor.backward();
			rightMotor.backward();
		}

		// Let the motors go forward/backward until the sensor senses a black line.
		while (!Localization.isWithinInterval(colorSensor.getColorID(), 14, 0.5))
			; // do  nothing

		
		/*
		 * after crosssing, updating odoometer with the offset distance depending on whether x or y was crossed
		 */
		if (crossX) {
			odometer.setX(sensorOffset);
		} else {
			odometer.setY(sensorOffset);
		}
		
		// avoids the robot from reading multiple lines & also from over-crossing
		Delay.msDelay(delay);
		leftMotor.stop();
		rightMotor.stop();
	}

}
