package ca.mcgill.ecse211.lab4;

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

	private float color[];
	private static final double SQUARE_SIZE = 30.48;
	private float[] csData;
	private double correctX, correctY;
	double[] oldResult = new double [3];
	double oldSample;
	int passedLine;
	double newColor;
	int countx;
	int county;
	double dy;
	double dx;

	public LightLocalizer(Odometer odometer, Navigation nav) throws OdometerExceptions {
		this.odo = Odometer.getOdometer(Lab4.leftMotor, Lab4.rightMotor, Lab4.TRACK, Lab4.WHEEL_RAD);
		odo.setTheta(0);
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
		while(true) {
			//color sensor and scaling
			myColorSample.fetchSample(color, 0);
			newColor = csData[0];

			correctionStart = System.currentTimeMillis();

			// Trigger correction : store data in newColor
			myColorSample.fetchSample(color, 0);
			newColor = csData[0];

			// Store current robot position and current theta
			double[] result = odo.getXYT();
			double theta = result[2];

			//If line detected (intensity less than 0.3), only count once by keeping track of last value
			if((newColor) < 0.3 && oldSample > 0.3) {

				//Error handling 
				if(result != null) {

					//Beep to notify, update counter and find and set correct X and Y using old reference pts
					if(passedLine < 4) {
						passedLine++;
						Sound.beep();

					}
				}

				//Print to LCD
				Lab4.lcd.clear();
				String printThisTotal = "Lines passed: "+ passedLine;
				Lab4.lcd.drawString(printThisTotal, 0, 3);
				String printThisY = "Y passed: "+ county;
				Lab4.lcd.drawString(printThisY, 0, 4);
				String printThisX = "X passed: "+ countx;
				Lab4.lcd.drawString(printThisX, 0, 5);


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
			double[] result = odo.getXYT();
			//If line detected (intensity less than 0.3), only count once by keeping track of last value
			if((newColor) < 0.3 && oldSample > 0.3) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				break;
			}
			oldSample = newColor;
			
			Lab4.lcd.clear();
			String printThisColor = "color: "+ newColor;
			Lab4.lcd.drawString(printThisColor, 0, 2);
			String printThisX = "x: "+ result[0];
			Lab4.lcd.drawString(printThisX, 0, 3);
			String printThisY = "y: "+ result[1];
			Lab4.lcd.drawString(printThisY, 0, 4);
			String printThisTheta = "theta: "+ result[2];
			Lab4.lcd.drawString(printThisTheta, 0, 5);
		}
	}
}

