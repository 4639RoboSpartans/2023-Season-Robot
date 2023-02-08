package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.swerve.SwerveMovement;

public class Drive extends CommandBase {
    private final OI oi;
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    public Drive(SwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
        this.oi = oi;
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(swerveDriveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var rawMovement = getRawMovement();
        var swerveMovement = toRobotCentric(rawMovement);
        
        double wa1 = 0, wa2 = 0, wa3 = 0, wa4 = 0;

        // TODO: refactor these variable names to be more meaningful
        double A = swerveMovement.strideMovement - swerveMovement.rotationClockwise / Math.sqrt(2);
        double B = swerveMovement.strideMovement + swerveMovement.rotationClockwise / Math.sqrt(2);
        double C = swerveMovement.forwardMovement - swerveMovement.rotationClockwise / Math.sqrt(2);
        double D = swerveMovement.forwardMovement + swerveMovement.rotationClockwise / Math.sqrt(2);


        double ws1 = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        double ws2 = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double ws3 = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double ws4 = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        if (ws3 > 0.05 || ws3 < -0.05) {
            wa1 = Math.atan2(B, C) * 180 / Math.PI;
        }
        if (ws3 > 0.05 || ws3 < -0.05) {
            wa2 = Math.atan2(B, D) * 180 / Math.PI;
        }
        if (ws3 > 0.05 || ws3 < -0.05) {
            wa3 = Math.atan2(A, D) * 180 / Math.PI;
        }
        if (ws3 > 0.05 || ws3 < -0.05) {
            wa4 = Math.atan2(A, C) * 180 / Math.PI;
        }

        // 1 is FR, 2 is FL, 3 is RL, 4 is RR

        double max = ws1;
        if (ws2 > max)
            max = ws2;
        if (ws3 > max)
            max = ws3;
        if (ws4 > max)
            max = ws4;
        if (max > 1) {
            ws1 /= max;
            ws2 /= max;
            ws3 /= max;
            ws4 /= max;
        }
        // ws4 = ws3 = ws2 = ws1;
        SmartDashboard.putNumber("Speed", ws3);
        SmartDashboard.putNumber("Rotation", wa3);

        swerveDriveSubsystem.setModules(
            new SwerveModuleState(ws1, new Rotation2d(wa1)),
            new SwerveModuleState(ws2, new Rotation2d(wa2)),
            new SwerveModuleState(ws3, new Rotation2d(wa3)),
            new SwerveModuleState(ws4, new Rotation2d(wa4))
        );
    }

    public SwerveMovement getRawMovement(){
        double rawX = oi.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rawY = oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double rawRot = oi.getAxis(0, Constants.Axes.RIGHT_STICK_X);
        
        return new SwerveMovement(rawX, rawY, rawRot);
    }

    public SwerveMovement toRobotCentric(SwerveMovement movement) {
        SwerveMovement res = new SwerveMovement(movement);

        double heading = 0; //swerveDriveSubsystem.getHeadingRadians();

        res.forwardMovement = movement.strideMovement * Math.sin(heading) + movement.forwardMovement * Math.cos(heading);
        res.strideMovement  = movement.strideMovement * Math.cos(heading) - movement.forwardMovement * Math.sin(heading);

        return res;
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