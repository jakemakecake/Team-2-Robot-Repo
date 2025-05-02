package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * ShooterIO controls the shooter and intake motors.
 */
public class ShooterIO extends SubsystemBase {

    private final CANSparkMax shooterMotor;
    private final CANSparkMax intakeMotor;

    // Default speeds
    private double shooterSpeed = 0.0;
    private double intakeSpeed = 0.0;

    /**
     * Constructor for ShooterIO.
     *
     * @param shooterID CAN ID for the shooter motor
     * @param intakeID  CAN ID for the intake motor
     */
    public ShooterIO(int shooterID, int intakeID) {
        shooterMotor = new CANSparkMax(shooterID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);

        shooterMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();
    }

    /**
     * Set the speed of the shooter motor.
     *
     * @param speed Value between -1.0 and 1.0
     */
    public void setShooterSpeed(double speed) {
        shooterSpeed = speed;
        shooterMotor.set(speed);
    }

    /**
     * Set the speed of the intake motor.
     *
     * @param speed Value between -1.0 and 1.0
     */
    public void setIntakeSpeed(double speed) {
        intakeSpeed = speed;
        intakeMotor.set(speed);
    }

    /**
     * Run both shooter and intake motors.
     *
     * @param shooterSpeed Speed for shooter (-1.0 to 1.0)
     * @param intakeSpeed  Speed for intake (-1.0 to 1.0)
     */
    public void runShooterAndIntake(double shooterSpeed, double intakeSpeed) {
        setShooterSpeed(shooterSpeed);
        setIntakeSpeed(intakeSpeed);
    }

    /**
     * Stop both motors.
     */
    public void stopAll() {
        setShooterSpeed(0.0);
        setIntakeSpeed(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add logging or diagnostics here
        System.out.println("Shooter Speed: " + shooterSpeed + ", Intake Speed: " + intakeSpeed);
    }
}
