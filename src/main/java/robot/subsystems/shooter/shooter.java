package robot.subsystems.shooter;
// imports

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

  private final ShooterIO rightShooterMotor;
  private final ShooterIO leftShooterMotor;

  private final PIDController leftPidController = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0, 0);
  
  public Shooter(ShooterIO leftShooterMotor, ShooterIO rightShooterMotor) {

      this.leftShooterMotor = leftShooterMotor;
      this.rightShooterMotor = rightShooterMotor;



  }
 



}
