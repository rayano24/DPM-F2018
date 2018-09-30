package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

public class WallFollowingLab {

  // Parameters: adjust these for desired performance

  private static final int bandCenter = 34; // Offset from the wall (cm)
  private static final int bandWidth = 3; // Width of dead band (cm)
  private static final int motorLow = 105; // Speed of slower rotating wheel (deg/sec)
  private static final int motorHigh = 170; // Speed of the faster rotating wheel (deg/seec)
  private static final int collisionDistance = 17; // closest allowed distance to wall (cm) (determined by trial and error)
  private static final int bangBangConstant = 4; // increasing bandcenter without affecting PController (cm) (trial and error)
  private static final int longDistance = 55; // distance that is too far for the PController (cm) (trial and error)



  private static final Port usPort = LocalEV3.get().getPort("S2");
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  // Main entry point - instantiate objects used and set up sensor

  public static void main(String[] args) {

    int option = 0;
    Printer.printMainMenu(); // Set up the display on the EV3 screen
    while (option == 0) // and wait for a button press. The button
      option = Button.waitForAnyPress(); // ID (option) determines what type of control to use

    // Setup controller objects

    BangBangController bangbangController =
        new BangBangController(bandCenter, bandWidth, motorLow, motorHigh, collisionDistance, bangBangConstant, leftMotor,  rightMotor);

    PController pController = new PController(bandCenter, bandWidth, leftMotor, rightMotor, longDistance);

    // Setup ultrasonic sensor
    // There are 4 steps involved:
    // 1. Create a port object attached to a physical port (done already above)
    // 2. Create a sensor instance and attach to port
    // 3. Create a sample provider instance for the above and initialize operating mode
    // 4. Create a buffer for the sensor data

    @SuppressWarnings("resource") // Because we don't bother to close this resource
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // this instance
    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
                                                         // returned


    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, bangbangController);


    // Start the poller and printer threads
    usPoller.start();


  }
}
