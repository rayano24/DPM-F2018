package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private final int collisionDistance;
	private int distance;
	private final int FILTER_OUT = 25;
	private int filterControl = 0;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh, int collisionDistance,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.collisionDistance = collisionDistance;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(motorHigh); // Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// if we are TOO close to the wall, we want the robot to turn right in place and
		// readjust path so we set the right motor to backwards
		if (this.distance <= collisionDistance) {
			leftMotor.setSpeed(motorLow);
			rightMotor.setSpeed(motorHigh);

			leftMotor.forward();
			rightMotor.backward();
		}

		// too close to the wall but not as close as collisionDistance, we want it to turn
		// right, so we slow down the right motor
		else if (this.distance < bandCenter - bandwidth) {
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorLow);

			leftMotor.forward();
			rightMotor.forward();

		}

		// If it is too wall, we want it to turn left to become closer so we slow down
		// the left motor
		else if (this.distance > bandCenter + bandwidth) {
			leftMotor.setSpeed(motorLow);
			rightMotor.setSpeed(motorHigh);

			leftMotor.forward();
			rightMotor.forward();
		}

		// if it is the right distance from the wall, we just want both motors working
		// at the same power
		else {
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorLow);

			leftMotor.forward();
			rightMotor.forward();
		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
