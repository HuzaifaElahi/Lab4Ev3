package ca.mcgill.ecse211.lab4;

/**
 * This is the Ultrasonic Controller interface
 * Contains two methods which are implemented in NavWithObstacle
 */
public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
