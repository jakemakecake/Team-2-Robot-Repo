package robot.subsystems.shooter;


public interface ShooterIO {

    /**
     * sets the voltage for the motors
     * @param voltage
     */
    public void setVoltage(double voltage);

    /**
     * gets the velocity of the motors
     * @return
     */
    public double getVelocity();
}
