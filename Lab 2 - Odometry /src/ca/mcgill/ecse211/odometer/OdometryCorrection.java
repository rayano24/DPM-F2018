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
	private static final double SENSOR_OFFSET = 2.5;
	private final float brightnessTrigger = 0.14f;
	private Odometer odometer;
	private float[] colour = new float[sensorMode.sampleSize()];
	private boolean crossedLine = false;

	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final SensorMode sensorMode = colorSensor.getRGBMode();

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

			// TODO Trigger correction (When do I have information to correct?)
			// TODO Calculate new (accurate) robot position

			// TODO Update odometer with new calculated (and more accurate) vales

			// NEW
			double[] xyt = odometer.getXYT();
			double x = xyt[0];
			double y = xyt[1];
			double theta = xyt[2];

			// NEW
			sensorMode.fetchSample(colour, 0);
			float red = colour[0];
			float green = colour[1];
			float blue = colour[2];
			float brightness = red + green + blue;

			if (crossedLine) {
				if (!(brightness < brightnessTrigger)) {
					crossedLine = false;
					Sound.beep();

					// Determine general direction the robot is facing
					boolean movingUp = theta < 45 || theta > 335;
					boolean movingRight = Math.abs(theta - 270) < 45;
					boolean movingDown = Math.abs(theta - 180) < 45;
					boolean movingLeft = Math.abs(theta - 90) < 45;
					boolean movingVertically = movingUp || movingDown;
					boolean movingHorizontally = movingRight || movingLeft;

					double offset; // offset to account for distance between sensor and wheel center

					if (movingVertically) {
						// align to origin on first line
						if (!yOriginSet) {
							odometer.setY(-SENSOR_OFFSET);
							yOriginSet = true;
						} else {
							// account for offset depending on direction
							if (movingUp) {
								offset = -SENSOR_OFFSET;
							} else {
								offset = SENSOR_OFFSET;
							}

							// round y position to nearest tile length (plus offset)
							double yRounded = Math.round(y / TILE_SIZE) * TILE_SIZE + offset;
							odometer.setY(yRounded);
						}
					} else if (movingHorizontally) {
						// align to origin on first line
						if (!xOriginSet) {
							odometer.setX(SENSOR_OFFSET);
							xOriginSet = true;
						} else {
							// account for offset depending on direction
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
			} else {
				// Determine whether sensor detects black
				crossedLine = brightness < brightnessTrigger;
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
