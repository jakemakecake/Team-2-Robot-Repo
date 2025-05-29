package robot.subsystems.shooter;

public class shooterConstants {
    private final int LEFT_SHOOTER_MOTOR_ID = 1; //  change later to  CAN ID
    private final int RIGHT_SHOOTER_MOTOR_ID = 2; // change later to  CAN ID
    public final double Defaultvelocity = 0.7; // 70% (0-1) power, change as needed
    public final double MaxVelocity = 1; 
    
    public static final class PID {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }

      public static final class FF {
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
      }
      

}
