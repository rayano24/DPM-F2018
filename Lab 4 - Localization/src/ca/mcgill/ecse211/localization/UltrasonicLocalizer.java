package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class UltrasonicLocalizer implements Runnable {

	private static EV3LargeRegulatedMotor leftMotor = Localization.leftMotor;
	private static EV3LargeRegulatedMotor rightMotor = Localization.rightMotor;
	protected static SensorModes usSensor;

	private SampleProvider us;
	private float[] usData;

	protected static Odometer odometer;

	// angles at whicht he back and left walls are detected
	protected double backAngle;
	protected double leftAngle;

	private static final int rotateSpeed = 50;

	// trial and error to find the best number
	private static int d = 25;
	private static int k = 7;

	public UltrasonicLocalizer() {
		usSensor = Localization.usSensor;

		this.us = usSensor.getMode("Distance");
		this.usData = new float[us.sampleSize()];

		this.backAngle = 0;
		this.leftAngle = 0;

		odometer = Localization.odometer;
	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		if (Localization.isRising)
			risingEdge();
		else
			fallingEdge();

	}
	
	
	/**
	 * For the rising edge case. Takes advantage of turnTo from Navigation to rotate to the calculated angle.
	 */
	public void risingEdge() {

		// if not already in rising edge position
		
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);

		leftMotor.forward();
		rightMotor.backward();

		while (getDistance() > d + k)
			; // do nothing

		
		
		while (getDistance() > d - k)
			; // do nothing
		
		// now in the rising edge angle 

		double risingEdgeAngle = odometer.getXYT()[2];

		Sound.beep();

		// going out of the position

		leftMotor.backward();
		rightMotor.forward();
		
		// noise margin

		while (getDistance() < d + k)
			; // do nothing
		
		

		double outsideRisingEdgeAngle = odometer.getXYT()[2];

		Sound.beep();

		leftMotor.stop();
		rightMotor.stop();

		this.backAngle = (risingEdgeAngle + outsideRisingEdgeAngle) / 2;

		// rotate till rising edge
		
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);

		leftMotor.backward();
		rightMotor.forward();


		//TODO: HERE
		
		while (getDistance() > d - k)
			; // do nothing

		risingEdgeAngle = odometer.getXYT()[2];

		Sound.beep();

		// switch to fall edge
		leftMotor.forward();
		rightMotor.backward();

		while (getDistance() < d + k)
			; // do nothing

		outsideRisingEdgeAngle = odometer.getXYT()[2];

		leftMotor.stop();
		rightMotor.stop();

		this.leftAngle = (risingEdgeAngle + outsideRisingEdgeAngle) / 2; 

		Sound.beep();

		Localization.turnTo((odometer.getXYT()[2] + getHeadingChange() - 2) % 360);

		odometer.setTheta(0);

	}

	

	 /**
	  * For the falling edge case. Takes advantage of turnTo from Navigation to rotate to the calculated angle.
	  */
	public void fallingEdge() {
		
		// falling edge pos
		
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);

		leftMotor.forward();
		rightMotor.backward();

		while (getDistance() < d + k)
			;
		
		Delay.msDelay(1000);

		// look for a wall
		while (getDistance() > d + k)
			;

		double fallingEdgeAngle = odometer.getXYT()[2];

		Sound.beep();

		// get too close
		
		while (getDistance() > d - k)
			;

		double outsideFallingEdgeAngle = odometer.getXYT()[2];

		Sound.beep();

		leftMotor.stop();
		rightMotor.stop();

	
		this.backAngle = (fallingEdgeAngle + outsideFallingEdgeAngle) / 2;

		// rotate till outside rising edge
		
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);

		leftMotor.backward();
		rightMotor.forward();

		while (getDistance() < d + k)
			;

		Delay.msDelay(1000);

		// looking for the wall
		
		while (getDistance() > d + k)
			;

		fallingEdgeAngle = odometer.getXYT()[2];
		Sound.beep();

		// too close to wall, get angle
		while (getDistance() > d - k)
			;

		outsideFallingEdgeAngle = odometer.getXYT()[2];

		leftMotor.stop();
		rightMotor.stop();

		Sound.beep();

		this.leftAngle = (fallingEdgeAngle + outsideFallingEdgeAngle) / 2; //

		// +45 due to the 45 in heading change, not applicable in this case
		Localization.turnTo((odometer.getXYT()[2] + (getHeadingChange() + 45) - 2) % 360);

		odometer.setTheta(0);
	}

	/**
	 * 
	 * @return the distance reported by the US sensor
	 */
	// adapted from US Poller (lab 1 & lab 3)
	private int getDistance() {
		this.us.fetchSample(usData, 0); // acquire data
		int currentDistance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
		return currentDistance;
	}

	/**
	 * Get the heading change needed for turnTo
	 * @return the heading change 
	 */
	public double getHeadingChange() {
		double headingChange = 0;

		// get the difference between the calculated left angle and the current angle
		headingChange += (leftAngle - odometer.getXYT()[2]);

		// angleDifference is half the change in heading to get from the second angle
		// heading to the first angle heading, i.e. to reach the 45 degree heading
		double headingDifference = (backAngle - leftAngle + 360) * 0.5;
		
		return headingChange += headingDifference - 45;


	}

}
