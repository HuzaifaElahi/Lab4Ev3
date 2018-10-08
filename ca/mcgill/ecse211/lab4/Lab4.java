package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.UltrasonicLocalizer.LocalizationType;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {


	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.19;
	public static final double SQUARE_SIZE = 30.48;
	public static final double TRACK = 13.72;
	static Odometer odometer = null;

	//Motors and distance sensor
	static final Port usPort = LocalEV3.get().getPort("S1");
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	//Color Sensor and its variables are initialized
	private static final Port portColor = LocalEV3.get().getPort("S2");
	public static SensorModes myColor = new EV3ColorSensor(portColor);
	public static SampleProvider myColorSample = myColor.getMode("Red");
	static float[] sampleColor = new float[myColor.sampleSize()]; 


	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;

		do {
			lcd.clear();   		// clear the display

			// Ask the user whether map 1 or 2 / map 3 or 4 should be selected
			lcd.drawString("<      |      >", 0, 0);
			lcd.drawString("Falling|Rising ", 0, 1);
			lcd.drawString(" Edge  |  Edge ", 0, 2);
			lcd.drawString("       |       ", 0, 3);
			lcd.drawString("<      |      >", 0, 4);

			buttonChoice = Button.waitForAnyPress();      // Record choice (left or right press)

			// Until button pressed
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT); 


		try {
			odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		} catch (OdometerExceptions e) {
			System.out.println("Lab 4 line 59");
		}
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Navigation nav = new Navigation(leftMotor, rightMotor, odometer);
		
		//		nav.start();

		if (buttonChoice == Button.ID_LEFT) {
			UltrasonicLocalizer usl = new UltrasonicLocalizer(LocalizationType.FALLING_EDGE, odometer, nav);
			usl.fallingEdge();
		}
		else {
			UltrasonicLocalizer usl = new UltrasonicLocalizer(LocalizationType.RISING_EDGE, odometer, nav);
			usl.risingEdge();
		}





		// Wait before starting
		Button.waitForAnyPress();

	}

}