package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Odometer;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer implements UltrasonicController {
	private LocalizationType type;
	private Odometer odo;
	private Navigation nav;
	private static final int MOTOR_SPEED = 100;
	private static final int D_THRESHHOLD = 30;
	private static final int NOISE_MARGIN = 1;
	private static final int FILTER_OUT = 10;
	private static double ALPHA;
	private static double BETA;
	private static double ANGLE_CORRECTION;
	private int filterControl;

	private int distance;
	SensorModes usSensor = new EV3UltrasonicSensor(Lab4.usPort);                 // usSensor is the instance
	SampleProvider usDistance = usSensor.getMode("Distance");                    // usDistance provides samples 
	float[] usData = new float[usDistance.sampleSize()];                         // usData is the buffer for data
	UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, this);  // Instantiate poller

	public static enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	}

	public UltrasonicLocalizer(LocalizationType edge, Odometer odometer, Navigation nav) throws OdometerExceptions {
		this.type = edge;
		this.odo = Odometer.getOdometer(Lab4.leftMotor, Lab4.rightMotor, Lab4.TRACK, Lab4.WHEEL_RAD);
		Lab4.leftMotor.setSpeed(MOTOR_SPEED);
		Lab4.rightMotor.setSpeed(MOTOR_SPEED);
		this.nav = nav;
		usPoller.start();
	}

	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
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
		Lab4.lcd.clear();
		Lab4.lcd.drawString("distance: " + distance, 0, 3);
	}

	/**
	 * Performs falling edge localization
	 */
	void fallingEdge(){
		boolean isAboveThresh = false;
		double theta = 0;

		// Checks orientation or sets orientation to perform localization
		if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
			isAboveThresh = true;
		} else {
			findWallAbove();
			isAboveThresh = true;
		}

		// Find first falling edge
		while (true) {
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();

			if (isFalling() && isAboveThresh) {
				Navigation.leftMotor.stop(true);
                Navigation.rightMotor.stop(false);
				ALPHA = theta;
				isAboveThresh = false;
				break;
			}
		}

		// Find second falling edge
		while (true) {
			Navigation.leftMotor.backward();
			Navigation.rightMotor.forward();

			if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
				isAboveThresh = true;
			}

			if (isFalling() && isAboveThresh) {
				Navigation.leftMotor.stop(true);
                Navigation.rightMotor.stop(false);
				BETA = 360 + theta;
				break;
			}
		}

		if (ALPHA < BETA) {
			ANGLE_CORRECTION = 45 - ((ALPHA + BETA) / 2); 
		} else {
			ANGLE_CORRECTION = 225 - ((ALPHA + BETA) / 2);
		} 

		Navigation.turnTo(ANGLE_CORRECTION);
	}

	/**
	 * Performs rising edge localization
	 */
	void risingEdge(){
		boolean isBelowThresh = false;
		double theta = 0;

		// Checks orientation or sets orientation to perform localization
		if (readUSDistance() < (D_THRESHHOLD + NOISE_MARGIN)) {
			isBelowThresh = true;
		} else {
			findWallBelow();
			isBelowThresh = true;
		}

		// Find first rising edge
		while (true) {
			Navigation.leftMotor.forward();
		    Navigation.rightMotor.backward();

			if (isRising() && isBelowThresh) {
				Navigation.leftMotor.stop(true);
                Navigation.rightMotor.stop(false);
				ALPHA = theta;
				isBelowThresh = false;
				break;
			}
		}

		// Find second rising edge
		while (true) {
			Navigation.leftMotor.backward();
		    Navigation.rightMotor.forward();

			if (readUSDistance() < (D_THRESHHOLD + NOISE_MARGIN)) {
				isBelowThresh = true;
			}

			if (isFalling() && isBelowThresh) {
				Navigation.leftMotor.stop(true);
                Navigation.rightMotor.stop(false);
				BETA = 360 + theta;
				break;
			}
		}

		if (ALPHA < BETA) {
			ANGLE_CORRECTION = 45 - ((ALPHA + BETA) / 2); 
		} else {
			ANGLE_CORRECTION = 225 - ((ALPHA + BETA) / 2);
		} 

		Navigation.turnTo(ANGLE_CORRECTION);
	}

	/**
	 * Sets orientation of robot so it can perform the localization with falling edges 
	 */
	void findWallAbove() {
		while (true) {
			Navigation.leftMotor.forward();
		    Navigation.rightMotor.backward();

			if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
				Navigation.leftMotor.stop(true);
                Navigation.rightMotor.stop(false);
				break;
			}
		}
	}

	/**
	 * Sets orientation of robot so it can perform the localization with rising edges 
	 */
	void findWallBelow() {
		while (true) {
			Navigation.leftMotor.forward();
		    Navigation.rightMotor.backward();;

			if (readUSDistance() < (D_THRESHHOLD + NOISE_MARGIN)) {
				Navigation.leftMotor.stop(true);
                Navigation.rightMotor.stop(false);
				break;
			}
		}
	}

	boolean isFalling() {
		if (readUSDistance() < (D_THRESHHOLD - NOISE_MARGIN)) {
			return true;
		} else {
			return false;
		}
	}

	boolean isRising() {
		if (readUSDistance() > (D_THRESHHOLD + NOISE_MARGIN)) {
			return true;
		} else {
			return false;
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}