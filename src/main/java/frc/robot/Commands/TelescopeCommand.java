package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeCommand extends CommandBase {
    private final TelescopeSubsystem telescope;
    private final OI oi;
    private double pos ;
    public TelescopeCommand(TelescopeSubsystem telescope, OI oi){
        this.telescope = telescope;
        this.oi = oi;
        pos = 0;
        addRequirements(telescope);
    }
    @Override
    public void initialize() {
        telescope.setSpeed(0);
    }
    @Override
    public void execute() {
        // if(oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean()){
        //     pos = -25;
        // }
        if(oi.getButton(1, Constants.Buttons.LEFT_BUMPER).getAsBoolean()){
            MidPlace();
        }
        if(oi.getButton(1, Constants.Buttons.RIGHT_BUMPER).getAsBoolean()){
            HighPlace();
        }
        if(oi.getButton(1, Constants.Buttons.B_BUTTON).getAsBoolean()){
            LowPlace();
        }
        if(oi.getAxis(1, Constants.Axes.RIGHT_TRIGGER)>0.2){
            PlatformPickup();
        }
        if(oi.getAxis(1, Constants.Axes.LEFT_TRIGGER)>0.2){
            FloorPickup();
        }
        if(oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean()){
            Moving();
        }
        if(oi.getPovButton(1, 90).getAsBoolean()){
            pos+=2;
        }
        if(oi.getPovButton(1, 270).getAsBoolean()){
            pos-=2;
        }
        telescope.setMotorPos(pos);
        // telescope.setPosition(Position);
    }
    public void FloorPickup(){
        pos = Constants.SetPoints.FITelescope;
    }
    public void PlatformPickup(){
        pos = Constants.SetPoints.PITelescope;
    }
    public void Moving(){
        pos = Constants.SetPoints.MTelescope;
    }
    public void LowPlace(){
        pos = Constants.SetPoints.GSTelescope;
    }
    public void MidPlace(){
        pos = Constants.SetPoints.MCOTelescope;
    }
    public void HighPlace(){
        pos = Constants.SetPoints.TCOTelescope;
    }
}