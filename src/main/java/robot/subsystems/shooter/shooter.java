package robot.subsystems.shooter;
// imports

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private ShooterIO rightShooterMotor;
  private ShooterIO leftShooterMotor;
  
    public Shooter(ShooterIO leftShooterMotor, ShooterIO rightShooterMotor) {
      this.leftShooterMotor = leftShooterMotor;
      this.rightShooterMotor = rightShooterMotor;

  }
 



}
