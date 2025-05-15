package robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// imports

public class shooter extends SubsystemBase {
  // constants
  private final int LEFT_SHOOTER_MOTOR_ID = 1; //  change later to  CAN ID
  private final int RIGHT_SHOOTER_MOTOR_ID = 2; // change later to  CAN ID
  private final double SHOOTER_DEFAULT_SPEED = 0.7; // 70% (0-1) power, change as needed

  private final CANSparkMax leftShooterMotor;
  private final CANSparkMax rightShooterMotor;

  // constructor
  public shooter() {
    // initialize motors, code may change based on what type of shooter we use
    leftShooterMotor = new CANSparkMax(LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    // reset settings to default
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    rightShooterMotor.setInverted(true);

    leftShooterMotor.setSmartCurrentLimit(40);
    rightShooterMotor.setSmartCurrentLimit(40);

    // telemetry
    SmartDashboard.putNumber("Shooter Speed", SHOOTER_DEFAULT_SPEED);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Shooter Velocity", leftShooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(
        "Right Shooter Velocity", rightShooterMotor.getEncoder().getVelocity());
  }

  // spin up the shooter
  public void shoot() {
    double speed = SmartDashboard.getNumber("Shooter Speed", SHOOTER_DEFAULT_SPEED);
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  // set shooter speed
  public void setShooterSpeed(double speed) {
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  // stop shooter
  public void stop() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }

  // check if shooter is at target speed
  public boolean isAtTargetSpeed(double targetSpeed, double tolerance) {
    double currentSpeed = leftShooterMotor.getEncoder().getVelocity();
    return Math.abs(currentSpeed - targetSpeed) < tolerance;
  }
}
