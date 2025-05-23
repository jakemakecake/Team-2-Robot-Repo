package robot.subsystems.shooter;
// imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private ShooterIO rightShooterMotor;
  private ShooterIO leftShooterMotor;
  
    public Shooter(ShooterIO leftShooterMotor, ShooterIO rightShooterMotor) {
      this.leftShooterMotor = leftShooterMotor;
      this.rightShooterMotor = rightShooterMotor;
  }
 



}
