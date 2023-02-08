package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.navXSubsystem;
import frc.robot.util.math.math;
import frc.robot.util.swerve.SwerveMovement;

public class DriveCommand extends CommandBase {
    private final OI oi;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final navXSubsystem navX;

    public DriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, OI oi, navXSubsystem navX) {
        this.oi = oi;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.navX = navX;

        addRequirements(swerveDriveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var rawMovement = getRawMovement();
        var swerveMovement = toRobotCentric(rawMovement);
        
        double wa1 = 0, wa2 = 0, wa3 = 0, wa4 = 0;

        double A = swerveMovement.strideMovement()  - swerveMovement.rotation() / Math.sqrt(2);
        double B = swerveMovement.strideMovement()  + swerveMovement.rotation() / Math.sqrt(2);
        double C = swerveMovement.forwardMovement() - swerveMovement.rotation() / Math.sqrt(2);
        double D = swerveMovement.forwardMovement() + swerveMovement.rotation() / Math.sqrt(2);


        double wheelSpeedFrontLeft = math.magnitude(B, C);
        double wheelSpeedFrontRight = math.magnitude(B, D);
        double wheelSpeedBackLeft = math.magnitude(A, C);
        double wheelSpeedBackRight = math.magnitude(A, D);

        if (wheelSpeedBackLeft > 0.05 || wheelSpeedBackLeft < -0.05) {
            wa1 = Math.atan2(B, C) * 180 / Math.PI;
        }
        if (wheelSpeedBackLeft > 0.05 || wheelSpeedBackLeft < -0.05) {
            wa2 = Math.atan2(B, D) * 180 / Math.PI;
        }
        if (wheelSpeedBackLeft > 0.05 || wheelSpeedBackLeft < -0.05) {
            wa3 = Math.atan2(A, D) * 180 / Math.PI;
        }
        if (wheelSpeedBackLeft > 0.05 || wheelSpeedBackLeft < -0.05) {
            wa4 = Math.atan2(A, C) * 180 / Math.PI;
        }

        // 1 is FR, 2 is FL, 3 is RL, 4 is RR

        double max = wheelSpeedFrontLeft;
        if (wheelSpeedFrontRight > max)
            max = wheelSpeedFrontRight;
        if (wheelSpeedBackLeft > max)
            max = wheelSpeedBackLeft;
        if (wheelSpeedBackRight > max)
            max = wheelSpeedBackRight;
        if (max > 1) {
            wheelSpeedFrontLeft /= max;
            wheelSpeedFrontRight /= max;
            wheelSpeedBackLeft /= max;
            wheelSpeedBackRight /= max;
        }
        // ws4 = ws3 = ws2 = ws1;
        SmartDashboard.putNumber("Speed", wheelSpeedBackLeft);
        SmartDashboard.putNumber("Rotation", wa3);

        swerveDriveSubsystem.setModules(
            new SwerveModuleState(wheelSpeedFrontLeft, new Rotation2d(wa1)),
            new SwerveModuleState(wheelSpeedFrontRight, new Rotation2d(wa2)),
            new SwerveModuleState(wheelSpeedBackLeft, new Rotation2d(wa3)),
            new SwerveModuleState(wheelSpeedBackRight, new Rotation2d(wa4))
        );
    }

    public SwerveMovement getRawMovement(){
        double rawX = oi.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rawY = oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double rawRot = oi.getAxis(0, Constants.Axes.RIGHT_STICK_X);
        
        return new SwerveMovement(rawX, rawY, rawRot);
    }

    public SwerveMovement toRobotCentric(SwerveMovement movement) {
        double rawForwardMovement = movement.forwardMovement();
        double rawStrideMovement = movement.strideMovement();
        double rawRotation = movement.rotation();

        double heading = navX.readPitch(); //swerveDriveSubsystem.getHeadingRadians();

        double forwardMovement = rawStrideMovement * Math.sin(heading) + rawForwardMovement * Math.cos(heading);
        double strideMovement  = rawStrideMovement * Math.cos(heading) - rawForwardMovement * Math.sin(heading);

        return new SwerveMovement(forwardMovement, strideMovement, rawRotation);
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