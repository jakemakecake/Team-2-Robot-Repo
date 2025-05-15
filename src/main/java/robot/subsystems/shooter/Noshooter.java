package robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Noshooter extends SubsystemBase {
  public Noshooter() {}

  @Override
  public void periodic() {
    // No functionality here
  }

  public void shoot() {
    // Do nothing
  }

  public void stop() {
    // do nothing
  }

  public void setShooterSpeed(double speed) {
    // do nothing
  }
}
