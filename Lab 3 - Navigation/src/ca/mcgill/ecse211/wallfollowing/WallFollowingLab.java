package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

import java.util.concurrent.TimeUnit;

import ca.mcgill.ecse211.navigation.Navigation;
import lejos.hardware.Button;

public class WallFollowingLab implements Runnable {

	public static final EV3LargeRegulatedMotor leftMotor = Navigation.leftMotor;
	public static final EV3LargeRegulatedMotor rightMotor = Navigation.rightMotor;
	private static final int bandCenter = 34; // Offset from the wall (cm)
	private static final int bandWidth = 3; // Width of dead band (cm)
	private static final int motorLow = 105; // Speed of slower rotating wheel (deg/sec)
	private static final int motorHigh = 170; // Speed of the faster rotating wheel (deg/seec)
	private static final int collisionDistance = 17; // closest allowed distance to wall (cm) (determined by trial and error)
	private static final int bangBangConstant = 4; // increasing bandcenter without affecting PController (cm) (trial and error)
 
	public void run() {
		main(null);
	}

	public static void main(String[] args) {

		// Setup controller objects

		Controller obstacleController = new Controller(bandCenter, bandWidth, motorLow, motorHigh, collisionDistance,
				bangBangConstant, leftMotor, rightMotor);

		// Setup ultrasonic sensor
		// There are 4 steps involved:
		// 1. Create a port object attached to a physical port (done already above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating
		// mode
		// 4. Create a buffer for the sensor data

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = Navigation.usSensor; // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
																	// this instance
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
																// returned

		// Setup Ultrasonic Poller // This thread samples the US and invokes
		UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, obstacleController);

		// Start the poller
		usPoller.start();

	}

}
