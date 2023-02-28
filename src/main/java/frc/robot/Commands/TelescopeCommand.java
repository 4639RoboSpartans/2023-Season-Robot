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
        double position = oi.getAxis(1, Constants.Axes.RIGHT_STICK_Y)*0.2;

        telescope.setSpeed(position);
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