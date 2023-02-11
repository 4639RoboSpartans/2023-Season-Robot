package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final OI oi;
    
    public ElevatorCommand(ElevatorSubsystem elevator, OI oi) {
        this.elevator = elevator;
        this.oi = oi;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double speed = oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        elevator.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}