package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.wallfollowing.WallFollowingLab;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;


public class Navigation {
	
	public static final Port colorPort = LocalEV3.get().getPort("S1");
	public static final Port usPort = LocalEV3.get().getPort("S2");
	public static final EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	public static final SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final int forwardSpeed = 200; // regular forward speed
	public static final int approachSpeed = 100; // forward speed when approaching a waypoint for better precision
	public static final int rotateSpeed = 50;

	public static Odometer odometer;
	public static Display odometryDisplay;
	public static WallFollowingLab wallFollower;

}
