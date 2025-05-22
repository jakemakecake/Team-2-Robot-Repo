package robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Voltage;

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
    
    @Override
    public void setVoltage(double Voltage) {
        leftShooterMotor.setVoltage(Voltage);
    }

    @Override
    public double getVelocity() {
    }

	





   
    
    
    
    
    
    
    
    
    
    

}

