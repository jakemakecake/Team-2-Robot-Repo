package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ShooterIO controls the shooter and intake motors for intake and outtake functionality.
 */
public class ShooterIO extends SubsystemBase {

    private final CANSparkMax shooterMotor;
    private final CANSparkMax intakeMotor;

    // Current speed settings
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

        shooterMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Set the speed of the shooter motor.
     *
     * @param speed Value between -1.0 and 1.0
     */
    public void setShooterSpeed(double speed) {
        shooterSpeed = clampSpeed(speed);
        shooterMotor.set(shooterSpeed);
    }

    /**
     * Set the speed of the intake motor.
     *
     * @param speed Value between -1.0 and 1.0
     */
    public void setIntakeSpeed(double speed) {
        intakeSpeed = clampSpeed(speed);
        intakeMotor.set(intakeSpeed);
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
        // Outputs for diagnostics
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
    }

    /**
     * Clamp the motor speed to the safe range.
     *
     * @param speed Requested speed
     * @return Clamped speed between -1.0 and 1.0
     */
    private double clampSpeed(double speed) {
        return Math.max(-1.0, Math.min(1.0, speed));
    }
}
