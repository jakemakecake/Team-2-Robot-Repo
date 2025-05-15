package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.MotorType;

/**
 * ShooterIO controls the shooter and intake motors.
 */
public class ShooterIO extends SubsystemBase {

    private final CANSparkMax shooterMotor;
    private final CANSparkMax intakeMotor;

    private double shooterSpeed = 0.0;
    private double intakeSpeed = 0.0;

    public ShooterIO(int shooterID, int intakeID) {
        shooterMotor = new CANSparkMax(shooterID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);

        shooterMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();
    }

    public void setShooterSpeed(double speed) {
        shooterSpeed = speed;
        shooterMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeSpeed = speed;
        intakeMotor.set(speed);
    }

    public void runShooterAndIntake(double shooterSpeed, double intakeSpeed) {
        setShooterSpeed(shooterSpeed);
        setIntakeSpeed(intakeSpeed);
    }

    public void stopAll() {
        setShooterSpeed(0.0);
        setIntakeSpeed(0.0);
    }

    @Override
    public void periodic() {
        System.out.println("Shooter Speed: " + shooterSpeed + ", Intake Speed: " + intakeSpeed);
    }
}

