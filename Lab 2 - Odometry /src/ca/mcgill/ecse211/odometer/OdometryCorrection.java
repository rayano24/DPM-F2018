/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class OdometryCorrection implements Runnable {
	private static final double TILE_SIZE = 30.48;
	private static final long CORRECTION_PERIOD = 10;
	private static final double SENSOR_OFFSET = 0.5;
	private int yOffset = 4;
	private Odometer odometer;
	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final SensorMode redMode = colorSensor.getRedMode(); // measure intensity of reflected light
	private float[] colour = new float[redMode.sampleSize()];

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		boolean xOriginSet = false;
		boolean yOriginSet = false;

		while (true) {
			correctionStart = System.currentTimeMillis();

			// get odometer readings
			double x = odometer.getXYT()[0];
			double y = odometer.getXYT()[1];
			double theta = odometer.getXYT()[2];

			// get the intensity
			redMode.fetchSample(colour, 0);
			float sensorColour = colour[0];

			// if below 0.35 ( a number we determined), is on the black line
			if (sensorColour < 0.35) {
				Sound.beep();

				// Determine general direction the robot is facing
				boolean movingUp = theta < 45 || theta > 335;
				boolean movingDown = Math.abs(theta - 180) < 45;
				boolean movingLeft = Math.abs(theta - 90) < 45;
				boolean movingRight = Math.abs(theta - 270) < 45;

				boolean movingVertically = movingUp || movingDown;
				boolean movingHorizontally = movingRight || movingLeft;

				double offset; // offset to account for distance between sensor and wheel center

				if (movingVertically) {
					if (!yOriginSet) {
						odometer.setY(-SENSOR_OFFSET);
						yOriginSet = true;
					} else {
						// direction affects offset
						if (movingUp) {
							offset = -SENSOR_OFFSET;
						} else {
							offset = SENSOR_OFFSET;
						}

						// round y position to nearest tile length (plus offset)
						double yRounded = Math.round(y / TILE_SIZE) * TILE_SIZE + offset;
						odometer.setY(yRounded + yOffset);
					}
				} else if (movingHorizontally) {
					if (!xOriginSet) {
						odometer.setX(SENSOR_OFFSET);
						xOriginSet = true;
					} else {
						// offset varies based on direction
						if (movingRight) {
							offset = SENSOR_OFFSET;
						} else {
							offset = -SENSOR_OFFSET;
						}

						// round x position to nearest tile length (plus offset)
						double xRounded = Math.round(x / TILE_SIZE) * TILE_SIZE + offset;
						odometer.setX(xRounded);
					}
				}
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
