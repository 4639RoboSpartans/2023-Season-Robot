package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.ClawSubsystem;

public class CloseClawCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;

    public CloseClawCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
