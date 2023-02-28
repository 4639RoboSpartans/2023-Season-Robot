package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.Constants;
import frc.robot.OI;

public class WristCommand extends CommandBase{
    private final WristSubsystem wrist;
    private final OI oi;
    private double pos;
    public WristCommand(WristSubsystem wrist, OI oi){
        this.wrist = wrist;
        this.oi = oi;
        pos = 0;
        addRequirements(wrist);
    }
    @Override
    public void initialize(){
        wrist.stop();
    }
    @Override
    public void execute(){
        // wrist.setMotorPos(-1);//change

        if(oi.getButton(1, Constants.Buttons.X_BUTTON).getAsBoolean()){
            pos = Constants.SetPoints.MWrist;
    }
    
    if(oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean()){
            pos = Constants.SetPoints.TCOWrist;
    }
    if(oi.getButton(1, Constants.Buttons.B_BUTTON).getAsBoolean()){
            pos = Constants.SetPoints.GSWrist;
    }
    if(oi.getButton(1, Constants.Buttons.A_BUTTON).getAsBoolean()){
        // pos=;
        pos = Constants.SetPoints.MCOWrist;
    }
    
    wrist.setMotorPos(pos);
        // double speed = oi.getAxis(1, Constants.Axes.RIGHT_STICK_Y) * .2;
        // wrist.setSpeed(speed);
    }

    public void FloorPickup(){
        pos = Constants.SetPoints.FIWrist;
    }
    public void PlatformPickup(){
        pos = Constants.SetPoints.PIWrist;
    }
    public void Moving(){
        pos = Constants.SetPoints.MWrist;
    }
    public void LowPlace(){

    }
    public void MidPlace(){

    }
    public void HighPlace(){

    }

    @Override
    public void end(boolean interrupted){
        wrist.stop();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
