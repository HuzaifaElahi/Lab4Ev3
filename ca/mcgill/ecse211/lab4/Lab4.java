package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {

	  private static final int MOTOR_HIGH = 200; // Speed of the faster rotating wheel (deg/seec)
	  private static final int ROTATE_SPEED = 150;
	  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	  public static final double WHEEL_RAD = 2.2;
	  public static final double SQUARE_SIZE = 30.48;
	  public static final double TRACK = 13.72;
	  static Odometer odometer = null;
	  
	  //Motors and distance sensor
	  private static final Port usPort = LocalEV3.get().getPort("S1");
	  public static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	  public static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	  
	  //Color Sensor and its variables are initialized
	  private static final Port portColor = LocalEV3.get().getPort("S1");
	  public static SensorModes myColor = new EV3ColorSensor(portColor);
	  public static SampleProvider myColorSample = myColor.getMode("Red");
	  static float[] sampleColor = new float[myColor.sampleSize()]; 

	
	public static void main(String[] args) throws Exception {
		odometer = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Thread odoThread = new Thread(odometer);
	    odoThread.start();
	    Navigation navig = new Navigation();
		navig.start();
		Display LCD = new Display(lcd);
		
		// Wait before starting
		Button.waitForAnyPress();

	}

}
