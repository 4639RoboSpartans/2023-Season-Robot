package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final OI oi;
    private boolean pressedUp;
    private boolean pressedDown;
    private boolean endDown;
    
    public ElevatorCommand(ElevatorSubsystem elevator, OI oi) {
        pressedUp = false;
        pressedDown = false;
        endDown = false;
        this.elevator = elevator;
        this.oi = oi;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        // pressedUp = oi.getButton(0, Constants.Buttons.Y_BUTTON).getAsBoolean();
        // pressedDown = oi.getButton(0, Constants.Buttons.A_BUTTON).getAsBoolean();
        // endDown = oi.getButton(0, Constants.Buttons.B_BUTTON).getAsBoolean();
        // if(pressedUp) 
        //     elevator.move(0.5);
        // if(pressedDown) 
        //     elevator.move(-0.5);
        // if(endDown)
        //     elevator.move(0);

        double speed = oi.getPovButton(0, 90).getAsBoolean() ? 1 : oi.getPovButton(0, 270).getAsBoolean() ? -1 : 0;
        elevator.move(speed * .05);
        // elevator.setPosition(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}