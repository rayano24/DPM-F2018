package ca.mcgill.ecse211.localization;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import ca.mcgill.ecse211.odometer.*;

public class Localization {

	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static final Port cPort = LocalEV3.get().getPort("S1");
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3ColorSensor colorSensor = new EV3ColorSensor(cPort);
	public static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(usPort);
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static boolean isRising = false;
	
	public static Odometer odometer;

	
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 17.5;
	private static final double TILE_SIZE = 30.48;

	public static final int rotateSpeed = 50;
	public static final int forwardSpeed = 200;
	public static final int approachSpeed = 100; // this is the speed when approaching a waypoint
	
	
	public static void main(String[] args) throws OdometerExceptions {

		int option = 0;

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left   | Right >", 0, 0);
			lcd.drawString("         |        ", 0, 1);
			lcd.drawString(" Falling | Rising ", 0, 2);
			lcd.drawString("  Edge   |  Edge  ", 0, 3);

			option = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (option != Button.ID_LEFT && option != Button.ID_RIGHT);

		// Now, needs to do ultrasonic localization

		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Thread odometerThread = new Thread(odometer);
		odometerThread.start();

		Display display = new Display(lcd); // No need to change
		Thread displayThread = new Thread(display);
		displayThread.start();


		if (option == Button.ID_RIGHT) { // rising edge
			isRising = true;
		} else { // falling edge
			isRising = false;
		}
	

		UltrasonicLocalizer usLoc = new UltrasonicLocalizer();
		Thread usLocThread = new Thread(usLoc);
		usLocThread.start();

		// wait for action before continuing to light localization 
		
		Button.waitForAnyPress();

		LightLocalizer lightLoc = new LightLocalizer();
		Thread lightLocThread = new Thread(lightLoc);
		lightLocThread.start();
		
		Button.waitForAnyPress();
	    System.exit(0);
 

	}
	
	// The following is directly taken from our Navigation class in lab 3. (TravelTo, turnTo and its helper methods)
	
	/**
	 * Turns to the desired angle in theta
	 * @param theta
	 */
	public static void turnTo(double theta) {
		// Explanations for these if blocks are in the report.
		if (theta >= odometer.getXYT()[2]) {
			if (theta <= odometer.getXYT()[2] + 180) {
				rotate(theta, true);
			} else {
				rotate(theta, false);
			}
		} else {
			if (odometer.getXYT()[2] - theta < 180) {
				rotate(theta, false);
			} else {
				rotate(theta, true);
			}
		}
	}
	

	/**
	 * Gets the needed angle and moves towards it. Slows down when target distance
	 * is less than 2.
	 * 
	 * @param y the target y position
	 * @param x the target x position
	 */
	public static void travelTo(double x, double y) {

		x *= TILE_SIZE;
		y *= TILE_SIZE;

		double currentX = odometer.getXYT()[0];
		double currentY = odometer.getXYT()[1];

		double firstRotateValue = 0;
		double secondRotateValue = 0;
		double denominator = calculateDistance(currentX, currentY, x, y);
		double numerator = 0;
		double targetDistance;

		// denominator = distance between current position/target
		// numerator/denominator

		// logic is to find the cos(theta) = adj/hyp

		if (x >= currentX && y >= currentY) { // top right quadrant
			secondRotateValue = 1;
			numerator = Math.abs(y - currentY);
			secondRotateValue *= Math.toDegrees(Math.acos(numerator / denominator));

		} else if (x >= currentX && y < currentY) { // bottom right
			firstRotateValue = 90;
			secondRotateValue = 1;
			numerator = Math.abs(x - currentX);
			secondRotateValue *= Math.toDegrees(Math.acos(numerator / denominator));

		} else if (x < currentX && y >= currentY) { // top left
			secondRotateValue = -1;
			numerator = Math.abs(y - currentY);
			secondRotateValue *= Math.toDegrees(Math.acos(numerator / denominator));

		} else { // bottom left
			firstRotateValue = -90;
			secondRotateValue = -1;
			numerator = Math.abs(x - currentX);
			secondRotateValue *= Math.toDegrees(Math.acos(numerator / denominator));
		}

		// turn to the angle
		double turnAngle = (firstRotateValue + secondRotateValue + 360) % 360;
		turnTo(turnAngle);

		currentX = odometer.getXYT()[0];
		currentY = odometer.getXYT()[1];

		targetDistance = calculateDistance(currentX, currentY, x, y);

		// this is to manage our speeds

		// While the distance between the current position and the target is more than 2
		while (targetDistance > 2) {
			lcd.drawString("distToTarget:" + targetDistance, 0, 5);
			if (targetDistance > 10) {
				leftMotor.setSpeed(forwardSpeed);
				rightMotor.setSpeed(forwardSpeed);

				// here we want to approach the waypoint at a lower speed to ensure precision
			} else if (targetDistance >= 3 && targetDistance <= 10) {
				leftMotor.setSpeed(approachSpeed);
				rightMotor.setSpeed(approachSpeed);
			} else {
				break;
			}
			leftMotor.forward();
			rightMotor.forward();

			currentX = odometer.getXYT()[0];
			currentY = odometer.getXYT()[1];
		}

		leftMotor.stop();
		rightMotor.stop();
	}

	
	/**
	 * Rotates towards the targetAngle
	 * @param targetAngle
	 * @param isTurningRight
	 */
	public static void rotate(double targetAngle, boolean turnRight) {
		// turn right relative to current heading!

		lcd.drawString("target angle:" + targetAngle, 0, 4);
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed - 10);

		do {

			if (turnRight) {
				leftMotor.forward(); // turning right
				rightMotor.backward();
			} else {
				leftMotor.backward(); // left
				rightMotor.forward();
			}
		} while (!isWithinInterval(odometer.getXYT()[2], targetAngle, 0.5));

		while (!isWithinInterval(odometer.getXYT()[2], targetAngle, 0.5)) {

			turnRight = (!(odometer.getXYT()[2] > targetAngle));

			if (turnRight) {
				leftMotor.forward();
				rightMotor.backward();
			} else {
				leftMotor.backward();
				rightMotor.forward();
			}
		}
		leftMotor.stop();
		rightMotor.stop();

	}

	public static boolean isWithinInterval(double value, double center, double offset) {
		return value >= center - offset && value <= center + offset;
	}

	public static double calculateDistance(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
	}
	
	
	

}
