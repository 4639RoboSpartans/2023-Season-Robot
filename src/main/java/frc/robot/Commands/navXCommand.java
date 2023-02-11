package frc.robot.commands;
//if angle is not flat go "uphill"
//if angle is flat for 3 seconds, isFinisehd returns true

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//to move forward/backwards do: swerve.setAllState(new SwerveModuleState(speed, new Rotation2d()))
//getPitch() - angle of the front of the robot off the ground 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.navXSubsystem;

public class navXCommand extends CommandBase{
    private final SwerveDriveSubsystem swerve;
    private final navXSubsystem navx;
    public navXCommand(SwerveDriveSubsystem swerve, navXSubsystem navx){
        this.swerve = swerve;
        this.navx = navx;
        addRequirements(swerve);
        Timer.getFPGATimestamp();
        

    }
    public void initialize(){

    }
    public void execute(){

        if(!navx.isZero()){
            double speed = Math.max(Math.min(navx.readPitch()/16, 1), -1);
            SmartDashboard.putNumber("Motor Value", speed);
            
            double rot = speed > 0 ? 180 : 0;
            speed = Math.abs(speed);

            swerve.setModules(new SwerveModuleState(speed, Rotation2d.fromDegrees(rot)));
        }   
        else{
            swerve.setModules(new SwerveModuleState());
            Timer.getFPGATimestamp();

        }
        

     }
    public void end(){

    }
    public boolean isFinished(){
        return false;
    }
    
}
