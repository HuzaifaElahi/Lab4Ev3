package ca.mcgill.ecse211.lab4;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread implements Runnable {
	public static SensorModes myColor = new EV3ColorSensor(Lab4.portColor);
	public static SampleProvider myColorSample = myColor.getMode("Red");
	static float[] sampleColor = new float[myColor.sampleSize()];
	private Odometer odo;
	private Navigation nav;
	static double []result = new double[3];
	private float color[];
	private static final double SQUARE_SIZE = 30.48;
	private static final int SENSOR_OFFSET = 13;
	static ArrayList<Double> points = new ArrayList<Double>();
	private float[] csData;
	double[] oldResult = new double [3];
	double oldSample;
	static int passedLine;
	static double newColor;
	private static final double D = 4;
	static double xOffset = 0;
	static double yOffset = 0;
	double dy;
	double dx;

	public LightLocalizer(Odometer odometer, Navigation nav) throws OdometerExceptions {
		this.odo = Odometer.getOdometer(Lab4.leftMotor, Lab4.rightMotor, Lab4.TRACK, Lab4.WHEEL_RAD);
		odo.setTheta(0);
		result = odo.getXYT();
		Lab4.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Lab4.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		this.nav = nav;
		color = new float[myColorSample.sampleSize()];
		this.csData = color;
	}

	@Override
	public void run() {
		odo.setTheta(0);
		try {
			goToOrigin();
		} catch (OdometerExceptions e1) {
		}
		getLocalizationPts();
		Lab4.isLightLocalizing = false;
		try {
			performLocalization();
		} catch (OdometerExceptions e) {
		}

	} 

	private void performLocalization() throws OdometerExceptions {
		Lab4.isLightLocalizingTurn = true;
		double yp = points.get(0);
		double xp = points.get(1);
		double yn = points.get(2);
		double xn = points.get(3);

		xOffset = -D * Math.cos((yn - yp) / 2);
		yOffset = -D * Math.abs(Math.cos((xn - xp) / 2));

		// correct the odometer
		odo.setX(xOffset);
		odo.setY(yOffset);

		// this makes sure it travels to the true origin
		Navigation.turnTo(odo.getXYT()[2]+(90-((yn-yp)-180)+(yn-yp)/2));
		//Navigation.turnTo(0);
		Navigation.travelToHypot(0, 0);
		Lab4.leftMotor.stop(true);
		Lab4.rightMotor.stop(false);


	}

	private void getLocalizationPts() {
		long correctionStart, correctionEnd;
		double currentOdo = odo.getXYT()[2];
		Lab4.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Lab4.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.leftMotor.backward();
		Navigation.rightMotor.forward();

		while(true) {
			//color sensor and scaling
			myColorSample.fetchSample(color, 0);
			newColor = csData[0];
			correctionStart = System.currentTimeMillis();
			// Store current robot position and current theta
			result = odo.getXYT();
			//If line detected (intensity less than 0.3), only count once by keeping track of last value
			if((newColor) < 0.3 && oldSample > 0.3) {
				//Error handling 
				if(result != null) {
					//Beep to notify, update counter and find and set correct X and Y using old reference pts
					passedLine++;
					points.add(result[2]);
					Sound.beep();
				}
			}
			//Store color sample
			oldSample = newColor;
			
			if(passedLine > 0) {
				if(result[2] > currentOdo - 5 && result[2] < currentOdo + 5) {
					break;
				}
			}
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < 10) {
				try {
					Thread.sleep(10 - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
		Navigation.leftMotor.stop(true);
		Navigation.rightMotor.stop(false);
	}

	private void goToOrigin() throws OdometerExceptions {

		Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);	//Weaker motor compensation
		Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);

		Navigation.leftMotor.rotate(Navigation.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45.0), true);
		Navigation.rightMotor.rotate(-Navigation.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45.0), false);

		// turn 90 degrees clockwise
		Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
		Navigation.leftMotor.forward();
		Navigation.rightMotor.forward();
		while(true) {
			//color sensor and scaling
			myColorSample.fetchSample(color, 0);
			newColor = csData[0];
			result = odo.getXYT();
			//If line detected (intensity less than 0.3), only count once by keeping track of last value
			if((newColor) < 0.3 && oldSample > 0.3) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				Navigation.leftMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
				Navigation.rightMotor.setSpeed(UltrasonicLocalizer.MOTOR_SPEED);
				Navigation.leftMotor.rotate(-Navigation.convertDistance(Lab4.WHEEL_RAD, SENSOR_OFFSET), true);
				Navigation.rightMotor.rotate(-Navigation.convertDistance(Lab4.WHEEL_RAD, SENSOR_OFFSET), false);
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				break;
			}
			oldSample = newColor;

		}
	}
}

