package ca.mcgill.ecse211.wallfollowing;

import java.util.concurrent.TimeUnit;

import ca.mcgill.ecse211.navigation.Navigation;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Controller implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private final int collisionDistance;
	private final int bangBangConstant;
	private int distance;
	private final int FILTER_OUT = 25;
	private int filterControl = 0;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public Controller(int bandCenter, int bandwidth, int motorLow, int motorHigh, int collisionDistance,
			int bangBangConstant, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.collisionDistance = collisionDistance;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.bangBangConstant = bangBangConstant;
		leftMotor.setSpeed(motorHigh); // Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {

		if (distance < 15) {
			// the rotate used is from the motor class, not navigation class
			// turrn right

			leftMotor.rotate(convertAngle(Navigation.WHEEL_RAD, Navigation.TRACK, 90.0), true);
			rightMotor.rotate(-convertAngle(Navigation.WHEEL_RAD, Navigation.TRACK, 90.0), false);

			leftMotor.setSpeed(Navigation.forwardSpeed);
			rightMotor.setSpeed(Navigation.forwardSpeed);

			leftMotor.rotate(convertDistance(Navigation.WHEEL_RAD, 25.0), true);
			rightMotor.rotate(convertDistance(Navigation.WHEEL_RAD, 25.0), false);
			
			try {
				TimeUnit.SECONDS.sleep(3);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}


			// Turn left
			leftMotor.rotate(-convertAngle(Navigation.WHEEL_RAD, Navigation.TRACK, 90.0), true);
			rightMotor.rotate(convertAngle(Navigation.WHEEL_RAD, Navigation.TRACK, 90.0), false);

			leftMotor.setSpeed(Navigation.forwardSpeed);
			rightMotor.setSpeed(Navigation.forwardSpeed);

			leftMotor.rotate(convertDistance(Navigation.WHEEL_RAD, 50.0), true);
			rightMotor.rotate(convertDistance(Navigation.WHEEL_RAD, 50.0), false);
			
			try {
				TimeUnit.SECONDS.sleep(3);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			// turn left
			leftMotor.rotate(-convertAngle(Navigation.WHEEL_RAD, Navigation.TRACK, 90.0), true);
			rightMotor.rotate(convertAngle(Navigation.WHEEL_RAD, Navigation.TRACK, 90.0), false);

			leftMotor.setSpeed(Navigation.forwardSpeed);
			rightMotor.setSpeed(Navigation.forwardSpeed);

			leftMotor.rotate(convertDistance(Navigation.WHEEL_RAD, 25.0), true);
			rightMotor.rotate(convertDistance(Navigation.WHEEL_RAD, 25.0), false);
			
			try {
				TimeUnit.SECONDS.sleep(1);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			
			Navigation.travelTo(Navigation.waypoints[Navigation.completedWayPoints]);

		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	// these are taken from square driver
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
