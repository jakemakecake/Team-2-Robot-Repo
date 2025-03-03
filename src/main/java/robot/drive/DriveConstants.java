package robot.drive;

public class DriveConstants {

    public static final double WHEEL_RADIUS = 0.08; //Meters
    public static final double CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS;
    public static final double GEARING = 8.0;
    //dist
    public static final double POSITION_FACTOR = CIRCUMFERENCE * GEARING;
    //*meters/s*
    public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;
    
    public static final class FF {
        public static final double kS = 1;
        public static final double kV = 3;
      }

    public static final class PID {
        public static final double kP = 8.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }

    public static final double MAX_SPEED = 2; // Meters per second
}
