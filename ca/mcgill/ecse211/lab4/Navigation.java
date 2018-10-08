package ca.mcgill.ecse211.lab4;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


/**
 * This is the Navigation class which extends Thread and implements
 * Runnable and Ultrasonic Controller. It uses the Ultrasonic sensor
 * to handle cases where an object is detected in the path of the robot 
 * and otherwise travels to the selected waypoints that are to it by
 * Controller in the NavWithObstacle constructor
 * 
 * @author Huzaifa, Jake
 * 
 */
public class Navigation {

    
    // Parameters: Can adjust these for desired performance
    private static final int MOTOR_HIGH = 100;     // Speed of the faster rotating wheel (deg/seec)
    private static final int ROTATE_SPEED = 100;   // Speed upon rotation
    private final double ODOMETER_ADJUSTMENT = 0.5;    // Adjusts the inaccuracy of the odometer

    //Motors initialized
    public static EV3LargeRegulatedMotor leftMotor;
    public static EV3LargeRegulatedMotor rightMotor;
    
    // Variables for odometer
    Odometer odometer = null;
    private static double prevAngle = 0;
    static boolean navigating = false;


    /**
     * Contructor, takes in and sets path passed by user
     * selection in Controller class
     * 
     * @param finalPath
     */
    public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
        Navigation.leftMotor = leftMotor;
        Navigation.rightMotor = rightMotor;
        try {
			this.odometer = Odometer.getOdometer(Navigation.leftMotor, Navigation.rightMotor, Lab4.TRACK, Lab4.WHEEL_RAD);
		} catch (OdometerExceptions e) {
			System.out.println("no odo");
		}
    }

    /**
     * This method makes robot move in the direction of the
     * waypoint whose coordinates are passed as arguments
     * 
     * @param x
     * @param y
     * @return void
     */
    public void travelTo(double x, double y) {
        // Define variables
        double odometer[] = { 0, 0, 0 }, absAngle = 0, deltaX = 0, deltaY = 0;

        // Set navigating to true
        navigating = true;

        // Get odometer readings
        try {
            odometer = Odometer.getOdometer().getXYT();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Convert X & Y coordinates to actual length (cm)
        x = x*Lab4.SQUARE_SIZE;
        y = y*Lab4.SQUARE_SIZE;

        // Set odometer reading angle as prevAngle
        prevAngle = odometer[2];

        // Get displacement to calculate new heading
        deltaX = x - odometer[0];
        deltaY = y - odometer[1];

        // Get absolute angle the robot must be facing
        absAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

        // If the value of absolute angle is negative, loop it back
        if (absAngle < 0)
            absAngle = 360 - Math.abs(absAngle);

        // Make robot turn to the absolute angle
        turnTo(absAngle);

        while (true) {
            
            // Set robot speed and move forward
            leftMotor.setSpeed(MOTOR_HIGH);
            rightMotor.setSpeed(MOTOR_HIGH);
            leftMotor.forward();
            rightMotor.forward();
             
            // Get odometer readings to check distance from finishing location 
            try {
              odometer = Odometer.getOdometer().getXYT();
            } catch (Exception e) {
              e.printStackTrace();
            }
          
            // Get new displacement to check if at destination and add correction for odometer
            deltaX = x - odometer[0] + ODOMETER_ADJUSTMENT;
            deltaY = y - odometer[1] + ODOMETER_ADJUSTMENT;
            
            // Check if the robot is at the coordinates of the destination
            if ((deltaX * deltaX) + (deltaY * deltaY) < .8) {
                leftMotor.stop(true);
                rightMotor.stop(false);
                navigating = false;
                return;
            }
        }
    }
    
    /**
     * This method causes the robot to turn (on point) to the absolute heading theta
     * 
     * @param theta
     * @return void
     */
    public static void turnTo(double theta) {
        boolean turnLeft = false;
        double deltaAngle;
        
        // Get change in angle we want
        deltaAngle = theta - prevAngle;
        
        // Turn counter-clockwise if negative theta
        if (deltaAngle < 0) {
          deltaAngle = Math.abs(deltaAngle);
          turnLeft = true;
        }
  
        
        // Set rotate speed
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);

        if (turnLeft) {
          leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaAngle), true);
          rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaAngle), false);
        } else {
          leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaAngle), true);
          rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, deltaAngle), false);
        }
        
        prevAngle = theta;
    }

    /**
     * This method returns the static boolean, navigating
     * 
     * @return boolean
     */
    public static boolean isNavigating() {
        return navigating;
    }

    /**
     * This method is a helper that allows the conversion of a distance to the total rotation of each wheel need to
     * cover that distance.
     * 
     * @param radius
     * @param distance
     * @return int
     */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
     * This method allows the conversion of an angle and it's calculated distance to the total rotation of each wheel need to
     * cover that distance.
     * 
     * @param radius
     * @param distance
     * @return int
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
}