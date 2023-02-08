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
import frc.robot.util.swerve.SwerveUtil;

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
        var swerveMovement = SwerveUtil.toRobotCentric(rawMovement, navX.getHeading());
        
        swerveDriveSubsystem.setMovement(swerveMovement);
    }

    public SwerveMovement getRawMovement(){
        double rawX = oi.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rawY = oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double rawRot = oi.getAxis(0, Constants.Axes.RIGHT_STICK_X);
        
        return new SwerveMovement(rawX, rawY, rawRot);
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