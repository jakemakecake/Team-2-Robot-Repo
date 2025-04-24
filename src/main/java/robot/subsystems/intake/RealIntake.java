package robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import robot.Ports.Intake;

public class RealIntake implements IntakeIO {
    private final VictorSP conveyorMotor;

    public RealIntake() {
        conveyorMotor = new VictorSP(Intake.INTAKE); // PWM port 0
    }
    public void setVoltage(double volts) {
        conveyorMotor.set(volts);
    }
}

