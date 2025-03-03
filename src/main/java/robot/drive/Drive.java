package robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import robot.Constants;
import robot.Ports;
import robot.Robot;
import robot.drive.DriveConstants.FF;
import robot.drive.DriveConstants.PID;


public class Drive extends SubsystemBase {
  private final CANSparkMax leftLeader = new CANSparkMax(Ports.DriveConstants.LEFT_LEADER, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(Ports.DriveConstants.LEFT_FOLLOWER, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(Ports.DriveConstants.RIGHT_LEADER, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(Ports.DriveConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  private final AnalogGyro gyro = new AnalogGyro(Ports.DriveConstants.GYRO_CHANNEL);

  private final DifferentialDriveOdometry odometry;
  private DifferentialDrivetrainSim driveSim;
  
    
    
    
      public Drive() {
        for (CANSparkMax spark : List.of(leftLeader, leftFollower, rightLeader, rightFollower)) {
          spark.restoreFactoryDefaults();
          spark.setIdleMode(IdleMode.kBrake);
    
        }
        rightFollower.follow(rightLeader);
        leftFollower.follow(leftLeader);
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    
        gyro.reset();
        
    
        odometry = new DifferentialDriveOdometry(
                new Rotation2d(), 
                0, 
                0, 
                new Pose2d());
    
        leftLeader.setInverted(true);
        
      }
    
    
    
      private void drive(double leftSpeed, double rightSpeed) {
        leftLeader.set(leftSpeed);
        rightLeader.set(rightSpeed);
        leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
        rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
    
        leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
        rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
    
        final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
        final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
      
        final double leftFeedforward = feedforward.calculate(realLeftSpeed);
        final double rightFeedforward = feedforward.calculate(realRightSpeed);
    
        final double leftPID = 
          leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
        final double rightPID = 
          rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);
    
        double leftVoltage = leftPID + leftFeedforward;
        double rightVoltage = rightPID + rightFeedforward;
    
        leftLeader.setVoltage(leftVoltage);
        rightLeader.setVoltage(rightVoltage);
        driveSim.setInputs(leftVoltage, rightVoltage);
  
        
    
    
    
      driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getMiniCIM(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.DRIVE_MASS,
            DriveConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            DriveConstants.STD_DEVS);
    //...

  }

  

  public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
    return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
  }
  
  private void updateOdometry(Rotation2d rotation) {
    odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
  }
  
  @Override 
  public void periodic() {
    updateOdometry(gyro.getRotation2d());
    updateOdometry(Robot.isReal() ? gyro.getRotation2d() :  
                        driveSim.getHeading());
    field2d.setRobotPose(pose());
    }
  @Override
  public void simulationPeriodic() {
    // sim.update() tells the simulation how much time has passed
    driveSim.update(Constants.PERIOD.in(Seconds));
    leftEncoder.setPosition(driveSim.getLeftPositionMeters());
    rightEncoder.setPosition(driveSim.getRightPositionMeters());
  }
  
  
  public Pose2d pose() {
    return odometry.getPoseMeters();
  }

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);

  private final PIDController leftPIDController =
      new PIDController(PID.kP, PID.kI, PID.kD);
  private final PIDController rightPIDController =
      new PIDController(PID.kP, PID.kI, PID.kD);
  
  @Log.NT 
  private final Field2d field2d = new Field2d();

}
  






