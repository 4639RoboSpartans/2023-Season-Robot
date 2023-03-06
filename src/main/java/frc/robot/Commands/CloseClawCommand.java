package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class CloseClawCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;

    public CloseClawCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        Constants.objectIn = true;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        Constants.LEDInfo.LEDStatus = 1;
        clawSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
