package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 25;
	private  final int steadyProportion = 2; // for speed correction, larger gives a smaller correction i.e, steady correction
	private final int rapidProportion = 1; // rapid correction proportion 

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	private int steadySpeedCorrection, rapidSpeedCorrection;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public PController(int bandCenter, int bandwidth, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		


		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 100 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 100) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 100: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
		
		int distError = this.distance - bandCenter;
		

		// TODO: process a movement based on the us distance passed in (P style)

		/* If we're too far from the wall, we want to decrease the left motor speed to
		 make it turn left. Since it's very far, we will also want it to turn at a
		more slow pace. */
		if (distError >= 55) {

			steadySpeedCorrection = calcSpeed(steadyProportion, distError);

			leftMotor.setSpeed(MOTOR_SPEED - steadySpeedCorrection);
			rightMotor.setSpeed(MOTOR_SPEED + steadySpeedCorrection);

			leftMotor.forward();
			rightMotor.forward();

		}

		/* here, we apply the same logic as previously but we want it to turn at a faster rate to avoid collision  */
		else if (distError > 0 && distError < 55 ) {

			rapidSpeedCorrection = calcSpeed(rapidProportion, distError);

			leftMotor.setSpeed(MOTOR_SPEED - rapidSpeedCorrection);
			rightMotor.setSpeed(MOTOR_SPEED + rapidSpeedCorrection );

			leftMotor.forward();
			rightMotor.forward();

		}

		/* in this case, we are  close to the wall, however, not too close,so we apply a steady correction */
		else if (distError < 0 && distError > -10) {

			steadySpeedCorrection = calcSpeed(steadyProportion, distError);

			leftMotor.setSpeed(MOTOR_SPEED + steadySpeedCorrection);
			rightMotor.setSpeed(MOTOR_SPEED - steadySpeedCorrection);

			leftMotor.forward();
			rightMotor.forward();

		}

		/* in this case, we are too close to the wall so we apply a rapid correction. We also move the right motor backwards
		  so it can turn in its place */
		else if (distError <= -10) {

			rapidSpeedCorrection = calcSpeed(rapidProportion, distError);

			leftMotor.setSpeed((MOTOR_SPEED + rapidSpeedCorrection));
			rightMotor.setSpeed((MOTOR_SPEED - rapidSpeedCorrection));

			leftMotor.forward();
			rightMotor.backward();

		}
		
		/*
		 appropriate distance so we let it continue forward at the designated speed
		 */

		else {

			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);

			leftMotor.forward();
			rightMotor.forward();
		}

	}
	
	/**
	 * @return an int that is used to correct the motor speed
	 * @param proportion a varying int that depends on the distance from the weall
	 * @param distance distance - bandcenter
	 */

	int calcSpeed(int proportion, int distError) {

		int speedCorrection;
		int maxSpeed = 80;
		int correctionConstant = 4; // chosen because other numbers would be too large with the correction constants 2,1 and larger divisors would be messy

		int turnSpeed = correctionConstant / proportion; // decides how fast/slow you will turn

		
		// abs value because dist error could be negative
		speedCorrection = (int) (turnSpeed * (double) Math.abs(distError));

		if (speedCorrection >= maxSpeed) {
			speedCorrection = maxSpeed;

		}

		return speedCorrection;

	}
	

	@Override
	public int readUSDistance() {
		return this.distance;
	}



}
