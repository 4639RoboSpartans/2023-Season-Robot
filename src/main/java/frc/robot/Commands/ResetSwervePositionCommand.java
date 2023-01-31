package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.DriveSubsystem;

public class ResetSwervePositionCommand extends CommandBase {
    
    private DriveSubsystem driveSubsystem;

    public ResetSwervePositionCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //driveSubsystem.resetAnglesAndPositions();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //?riveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}