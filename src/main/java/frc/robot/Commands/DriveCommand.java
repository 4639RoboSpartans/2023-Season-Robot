package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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

    private double XYRest;
    private double RotRest;

    public DriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, OI oi, NavX navX) {
        XYRest = 0.6;
        RotRest = 0.4;
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
        //when X or Y button is pressed
        //robot rotation should be set to 180
        //it should align horizontally
        //it should align vertically

        //placing on reflective nodes
        if(oi.getButton(0, Buttons.X_BUTTON).getAsBoolean()){
            double tx=swerveDriveSubsystem.getRetroXoffset();
            double rotation = swerveDriveSubsystem.getRotation().getDegrees();
            if(tx<0)
            swerveDriveSubsystem.setMovement(tx, 90);
            else
            swerveDriveSubsystem.setMovement(-tx, 270);
        }
        //placing on april tag nodes
       else if(oi.getButton(0, Buttons.Y_BUTTON).getAsBoolean()){
            if(swerveDriveSubsystem.AprilTagDetected()){
                double locationx = 0;//subject to change
                double locationy =0; //subject to change
                double offsets[] = swerveDriveSubsystem.TagD3Coords();
                PIDController xyPID = new PIDController(0.01, 0, 0);
                PIDController rotPID = new PIDController(0.01, 0, 0);
                double xoff = offsets[0];
                double yoff = offsets[1];
                double angleOff = offsets[2]; //angle in degrees
                //scale xoff to xval, which is betweene -1 and 1;
                double xval = Math.min(12, xyPID.calculate(xoff, locationx))/12;
                double yval = Math.min(12, xyPID.calculate(yoff, locationy))/12;
                double rotval = Math.min(12, rotPID.calculate(angleOff, 0))/12;
                SwerveMovement rawMovement = new SwerveMovement(xval*XYRest, yval*XYRest, rotval*RotRest);
                swerveMovement = SwerveUtil.toRobotCentric(rawMovement, navX.getHeading());
                SmartDashboard.putString("swerve movement", swerveMovement.toString());
                swerveDriveSubsystem.setMovement(swerveMovement);
            }
            // swerveDriveSubsystem.setMovement(aprilHor, 90);
        }
        //taking off platform april tag LEFT
        else if (oi.getPovButton(0, 270).getAsBoolean()){
                //same as above but change location x to new offset
        }
        else if (oi.getPovButton(0, 90).getAsBoolean()){
                //same as above but change location x to new offset
        }
        //taking off platform april tag RIGHT

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
        if(oi.getPovButton(0, 0).getAsBoolean()){
            Constants.LEDInfo.LEDStatus = 2;
        }
        //set to Cone
        else if(oi.getPovButton(0, 180).getAsBoolean()){
            Constants.LEDInfo.LEDStatus = 3;
        }
    }

    public SwerveMovement getRawMovement(){
        double rawX = oi.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rawY =oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double rawRot = -oi.getAxis(0, Constants.Axes.RIGHT_STICK_X);
        // SmartDashboard.putNumber("RawRot", rawRot);
        return new SwerveMovement(rawY*XYRest , -rawX *XYRest, rawRot*RotRest);
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