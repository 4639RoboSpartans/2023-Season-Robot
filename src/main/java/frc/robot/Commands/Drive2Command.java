package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystem.DriveSubsystem;
import frc.robot.Constants;

public class Drive2Command extends CommandBase {
    private final OI oi;
    private final DriveSubsystem drive2Subsystem;

    public Drive2Command (OI oi, DriveSubsystem drive2Subsystem) {
        this.oi = oi;
        this.drive2Subsystem = drive2Subsystem;

        addRequirements(drive2Subsystem);
    }

    @Override
    public void execute() {
        double rawX = oi.getAxis(0, Constants.Axes.LEFT_STICK_X);
        double rawY = oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        double speed = Math.sqrt(rawX * rawX + rawY * rawY);
        double direction = Math.atan2(rawY, rawX);
        drive2Subsystem.setWheelSpeed(speed);
        drive2Subsystem.setDirection(direction);
    }

    @Override
    public void end(boolean interrupted) {
        drive2Subsystem.setWheelSpeed(0);
        drive2Subsystem.setDirection(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
