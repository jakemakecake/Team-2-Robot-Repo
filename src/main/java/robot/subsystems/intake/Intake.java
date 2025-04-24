package robot.subsystems.intake;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Robot;

public class Intake extends SubsystemBase {
    private final IntakeIO hardware;

    public Intake(){
        if (Robot.isReal()) { 
            hardware = new RealIntake();
        } else {
            hardware = new NoIntake();
        }
    }

    public Command stop() {
        return runOnce(() -> hardware.setVoltage(0));
    } 
}