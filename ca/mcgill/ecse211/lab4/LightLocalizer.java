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
	ArrayList<Double> points = new ArrayList<Double>();
	private float[] csData;
	private double correctX, correctY;
	double[] oldResult = new double [3];
	double oldSample;
	int passedLine;
	static double newColor;
	int countx;
	int county;
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
		long correctionStart, correctionEnd;
		try {
			goToOrigin();
		} catch (OdometerExceptions e1) {
		}
		doLocalization();
	} 

	private void doLocalization() {
		long correctionStart, correctionEnd;
		while(true) {
			//color sensor and scaling
			myColorSample.fetchSample(color, 0);
			newColor = csData[0];
			correctionStart = System.currentTimeMillis();
			// Trigger correction : store data in newColor
			myColorSample.fetchSample(color, 0);
			newColor = csData[0];
			// Store current robot position and current theta
			result = odo.getXYT();
			double theta = result[2];
			//If line detected (intensity less than 0.3), only count once by keeping track of last value
			if((newColor) < 0.3 && oldSample > 0.3) {
				//Error handling 
				if(result != null) {
					//Beep to notify, update counter and find and set correct X and Y using old reference pts
					if(passedLine < 4) {
						passedLine++;
						points.add(result[2]);
						Sound.beep();
					}
				}

				//Set new correct XYT and store info for next loop
				oldResult[0] = correctX;
				oldResult[1] = correctY;
				oldResult[2] = theta;
				oldSample = newColor;
			}
			//Store color sample
			oldSample = newColor;
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

