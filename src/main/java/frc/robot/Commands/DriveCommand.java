package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants;

public class DriveCommand extends CommandBase {
    private final OI oi;
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    public DriveCommand (SwerveDriveSubsystem drive2Subsystem, OI oi) {
        this.oi = oi;
        this.swerveDriveSubsystem = drive2Subsystem;

        addRequirements(drive2Subsystem);
    }

    @Override
    public void execute() {
        double rawX = oi.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rawY = oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double speed = Math.sqrt(rawX * rawX + rawY * rawY);
        double direction = Math.atan2(rawY, rawX);
        
        swerveDriveSubsystem.setAllModules(new SwerveModuleState(speed, Rotation2d.fromRadians(direction)));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.setAllModules(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
