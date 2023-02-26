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
    private double pos;
    public ElevatorCommand(ElevatorSubsystem elevator, OI oi) {
        pressedUp = false;
        pressedDown = false;
        endDown = false;
        this.elevator = elevator;
        this.oi = oi;
        pos = 0;
        addRequirements(elevator);
    }

    @Override
    public void execute() {

        // elevator.setMotorPos(50);//change

         if(oi.getButton(1, Constants.Buttons.X_BUTTON).getAsBoolean()){
                pos = Constants.SetPoints.MElevator;
        }
        
        if(oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean()){
                pos = Constants.SetPoints.TCOElevator;;
        }
        if(oi.getButton(1, Constants.Buttons.B_BUTTON).getAsBoolean()){
                pos = Constants.SetPoints.GSElevator;
        }
        if(oi.getButton(1, Constants.Buttons.A_BUTTON).getAsBoolean()){
            // pos=;
            pos = Constants.SetPoints.MCOElevator;
    }
        elevator.setMotorPos(pos);

        // double position = oi.getAxis(1, Constants.Axes.RIGHT_STICK_Y)*0.175;

        // elevator.setSpeed(position);
        // telescope.setPosition(Position);
        //pressedUp = oi.getButton(0, Constants.Buttons.Y_BUTTON).getAsBoolean();
        //pressedDown = oi.getButton(0, Constants.Buttons.A_BUTTON).getAsBoolean();
        // if(pressedUp) 
        //     elevator.move(0.5);
        // if(pressedDown) 
        //     elevator.move(-0.5);
        // if(endDown)
        //     elevator.move(0);

        //double speed = pressedUp ? 1 : pressedDown ? -1 : 0;
        //elevator.move(speed * .125);
        // if(speed == 1) {
        //         elevator.setPosition(0.001);
        //         elevator.move();
        // }
        // if(speed == -1) {
        //         elevator.setPosition(0.0001);
        //         elevato//r.move();
        //}
        // elevator.setPosition(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}