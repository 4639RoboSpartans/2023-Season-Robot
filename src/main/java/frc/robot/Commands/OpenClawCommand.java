package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

public class OpenClawCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;

    public OpenClawCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        Constants.objectIn = false;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        Constants.LEDInfo.LEDStatus = 4;
        clawSubsystem.open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
