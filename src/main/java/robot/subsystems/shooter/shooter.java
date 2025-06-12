package robot.subsystems.shooter;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
// imports

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.subsystems.shooter.shooterConstants.FF;
import robot.subsystems.shooter.shooterConstants.PID;
import robot.subsystems.shooter.shooterConstants;

public class shooter extends SubsystemBase {
  private final PIDController leftpid = new PIDController(PID.kP, PID.kI, PID.kD);
  private final PIDController rightpid = new PIDController(PID.kP, PID.kI, PID.kD);
  private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(FF.kS, FF.kA, FF.kV);
  private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(FF.kS, FF.kA, FF.kV);


  private final ShooterIO rightShooterMotor;
  private final ShooterIO leftShooterMotor;

  private final PIDController leftPidController = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0, 0);
  
    public void Shooter(ShooterIO leftShooterMotor, ShooterIO rightShooterMotor) {
      this.leftShooterMotor = leftShooterMotor;
      this.rightShooterMotor = rightShooterMotor;
      setDefaultCommand(run(()-> update(0)));
      
  }
 
  public void update(double VelocitySetPoint)
  {
    double velocity =
      Double.isNaN(VelocitySetPoint)
      ? Defaultvelocity,
      : MathUtil.clamp(VelocitySetPoint,-MaxVelocity, MaxVelocity)



  }
}