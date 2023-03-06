package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.Buttons;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.NavX;
import frc.robot.swerve.SwerveMovement;
import frc.robot.swerve.SwerveUtil;

public class DriveCommand extends CommandBase {
    //private LimeLight light;
    private final OI oi;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final NavX navX;

    public DriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, OI oi, NavX navX) {
        this.oi = oi;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.navX = navX;
        addRequirements(swerveDriveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SwerveMovement swerveMovement;
        // if(LimeLight.getZDistance() != -1) {
        //     swerveMovement = SwerveUtil.toRobotCentric(new SwerveMovement(LimeLight.getZDistance(), LimeLight.getXDistance(), LimeLight.getXRotation()), navX.getHeading());
        // }
        // else {

            //4 different vision aligments: Reflective tape, april tag placement, april tag intake left, april tag intake right

        
        // double aprilHor = swerveDriveSubsystem.getAprilXOffset();
        double leftOffset=0;
        double rightOffset=0;
        if(oi.getButton(0, Buttons.X_BUTTON).getAsBoolean()){
            double tx=swerveDriveSubsystem.getRetroXoffset();
            if(tx<0)
            swerveDriveSubsystem.setMovement(tx, 90);
            else
            swerveDriveSubsystem.setMovement(-tx, 270);
        }
       else if(oi.getButton(0, Buttons.Y_BUTTON).getAsBoolean()){
            double aprilHor = swerveDriveSubsystem.getAprilXOffset();
            if(aprilHor<0)
            swerveDriveSubsystem.setMovement(aprilHor, 90);
            else
            swerveDriveSubsystem.setMovement(-aprilHor, 270);
            // swerveDriveSubsystem.setMovement(aprilHor, 90);
        }
        // if(oi.getPovButton(0, 180).getAsBoolean()){
        //     swerveDriveSubsystem.setMovement(aprilHor+leftOffset, 90);
        // }
        // if(oi.getPovButton(0, 270).getAsBoolean()){
        //     swerveDriveSubsystem.setMovement(aprilHor+rightOffset, 90);
        // }
        else{
        var rawMovement = getRawMovement();
        swerveMovement = SwerveUtil.toRobotCentric(rawMovement, navX.getHeading());
        SmartDashboard.putString("swerve movement", swerveMovement.toString());
        swerveDriveSubsystem.setMovement(swerveMovement);
        }
        //set to Cube
        if(oi.getButton(0, Buttons.A_BUTTON).getAsBoolean()){
            Constants.LEDInfo.LEDStatus = 2;
        }
        //set to Cone
        else if(oi.getButton(0, Buttons.B_BUTTON).getAsBoolean()){
            Constants.LEDInfo.LEDStatus = 3;
        }
    }

    public SwerveMovement getRawMovement(){
        double rawX = oi.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rawY =oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double rawRot = -oi.getAxis(0, Constants.Axes.RIGHT_STICK_X);
        // SmartDashboard.putNumber("RawRot", rawRot);
        return new SwerveMovement(rawY*0.6 , -rawX *0.6, rawRot*0.4);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}