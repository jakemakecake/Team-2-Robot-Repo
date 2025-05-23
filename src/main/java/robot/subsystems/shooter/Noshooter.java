package robot.subsystems.shooter;

public class Noshooter implements ShooterIO {

  @Override
  public double getVelocity(){
    return 0;
  }
    


  @Override
  public void setVoltage(double voltage) {

  }

}
