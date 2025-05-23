package robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Realshooter implements ShooterIO{
    
 
//declaring hardware stuff
    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

//actual function
    public Realshooter(){
        leftShooterMotor = new CANSparkMax(10, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(11,MotorType.kBrushless);
        leftEncoder =  leftShooterMotor.getEncoder();
        rightEncoder =  rightShooterMotor.getEncoder();
        leftShooterMotor.setInverted(true);

    }

    /*
     * gets the left encoder's velocity
     */
    @Override
    public double getVelocity() {
        return leftEncoder.getVelocity();
    }

    /*
     * sets the voltage for both motors
     */
    @Override
    public void setVoltage(double voltage) {
        rightShooterMotor.setVoltage(voltage);
        leftShooterMotor.setVoltage(voltage);
      
    }

	





   
    
    
    
    
    
    
    
    
    
    

}

