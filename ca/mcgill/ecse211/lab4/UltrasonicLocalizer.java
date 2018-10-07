package ca.mcgill.ecse211.lab4;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer implements UltrasonicController {
	private int distance;
	SensorModes usSensor = new EV3UltrasonicSensor(Lab4.usPort);                      // usSensor is the instance
	SampleProvider usDistance = usSensor.getMode("Distance");                    // usDistance provides samples 
	float[] usData = new float[usDistance.sampleSize()];                         // usData is the buffer for data
	UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, this);  // Instantiate poller

	public static enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	}

	public UltrasonicLocalizer(LocalizationType fallingEdge, Odometer odometer, Navigation nav) {
		// TODO Auto-generated constructor stub
	}


	/**
	 * Performs falling edge localization
	 */
	void fallingEdge(){

	}

	/**
	 * Performs rising edge localization
	 */
	void risingEdge(){

	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
