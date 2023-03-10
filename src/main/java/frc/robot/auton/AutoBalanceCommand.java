package frc.robot.auton;
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
import frc.robot.swerve.SwerveMovement;
import frc.robot.swerve.SwerveUtil;
import frc.robot.math.vec2;
import frc.robot.subsystems.NavX;
//bruh
public class AutoBalanceCommand extends CommandBase{
    private final SwerveDriveSubsystem swerve;
    private final NavX navx;
    private double limit;
    public AutoBalanceCommand(SwerveDriveSubsystem swerve, NavX navx){
        this.swerve = swerve;
        this.navx = navx;
        limit = 10;
        addRequirements(swerve);
        Timer.getFPGATimestamp();
        

    }
    public void initialize(){
        // swerve.ClimbingMode();
    }
    public void execute(){

        if(!navx.isZero()){

            double pitchAngle = navx.getPitch()*(Math.PI/180);
            // if(navx)
            // double xSpeed = Math.max(Math.min(navx.getPitch()/27, 1), -1)*limit;
            double ySpeed = Math.sin(pitchAngle)*-1*limit;
            // double ySpeed = Math.max(Math.min(navx.getPitch()/27, 1), -1)*limit;
            SmartDashboard.putNumber("Speedupd", ySpeed);
            SwerveMovement movement = new SwerveMovement(ySpeed, 0, 0);

            swerve.setMovement(movement);
        }   
        else{
            swerve.setModulesStates(new SwerveModuleState());
            Timer.getFPGATimestamp();

        }
        

     }
    public void end(){
        swerve.drivingMode();
    }

    public boolean isFinished(){
        return false;
    }
}
