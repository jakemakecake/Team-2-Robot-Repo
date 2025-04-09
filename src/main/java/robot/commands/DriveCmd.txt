package robot.commands; 

import edu.wpi.first.wpilibj2.command.CommandBase;
//--------------------------------------------------
import robot.drive.DriveSubsystem;

public class DriveCmd extends CommandBase{

    private final DriveSubsystem driveSubsystem;
    private final double rightSpeed;
    private final double leftSpeed;

    public DriveCmd(DriveSubsystem driveSubsystem, double rightSpeed, double leftSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.rightSpeed = rightSpeed;
        this.leftSpeed = leftSpeed;
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveSubsystem.drive(leftSpeed, rightSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0); 

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

}
