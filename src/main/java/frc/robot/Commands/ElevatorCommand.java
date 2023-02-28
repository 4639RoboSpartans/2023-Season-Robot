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
                Moving();
        }
        
        if(oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean()){
                HighPlace();
        }
        if(oi.getButton(1, Constants.Buttons.B_BUTTON).getAsBoolean()){
                MidPlace();
        }
        if(oi.getButton(1, Constants.Buttons.A_BUTTON).getAsBoolean()){
            // pos=;
           LowPlace();
    }
        elevator.setMotorPos(pos);

    }

    public void FloorPickup(){
        pos = Constants.SetPoints.FIElevator;
    }
    public void PlatformPickup(){
        pos = Constants.SetPoints.PIElevator;
    }
    public void Moving(){
        pos = Constants.SetPoints.MElevator;
    }
    public void LowPlace(){
        pos = Constants.SetPoints.GSElevator;
    }
    public void MidPlace(){
        pos = Constants.SetPoints.MCOElevator;
    }
    public void HighPlace(){
        pos = Constants.SetPoints.TCOElevator;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}