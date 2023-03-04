package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ArmPivotSubsystem;

public class ArmCommand extends CommandBase {
    private final OI oi;
    private final ArmPivotSubsystem pivot;
    private double pos;
    private boolean objectIn;
    public ArmCommand(ArmPivotSubsystem pivot, OI oi){
        this.pivot= pivot;
        this.oi = oi;
        pos=0;
        objectIn = false;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.stop();
    }

    @Override
    public void execute() {
        // pivot.setVoltage(6*oi.getAxis(1, Constants.Axes.RIGHT_STICK_Y));
    //     if(oi.getButton(1, Constants.Buttons.X_BUTTON).getAsBoolean()){
    //         Moving();
    //         objectIn=false;
    //         // pos = -5;
    // }
    
    // if(oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean()){
    //        HighPlace();
    //        objectIn=true;
    //     // pos = -20;
    // }
    // if(oi.getButton(1, Constants.Buttons.B_BUTTON).getAsBoolean()){
    //         MidPlace();
    //         objectIn=true;
    //         // pos = -30;
    // }
    // if(oi.getButton(1, Constants.Buttons.A_BUTTON).getAsBoolean()){
    //     // pos=;
    //     LowPlace();
    //     objectIn=true;
    //     // pos = -60;
    // }
    
    if(oi.getButton(1, Constants.Buttons.LEFT_BUMPER).getAsBoolean()){
        MidPlace();
        objectIn = true;
    }
    if(oi.getButton(1, Constants.Buttons.RIGHT_BUMPER).getAsBoolean()){
        HighPlace();
        objectIn = true;
    }
    if(oi.getButton(1, Constants.Buttons.B_BUTTON).getAsBoolean()){
        LowPlace();
        objectIn = true;
    }
    if(oi.getAxis(1, Constants.Axes.RIGHT_TRIGGER)>0.2){
        PlatformPickup();
        objectIn = false;
    }
    if(oi.getAxis(1, Constants.Axes.LEFT_TRIGGER)>0.2){
        FloorPickup();
        objectIn = false;
    }
    if(oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean()){
        Moving();
        objectIn = true;
    }
    pivot.setMotorPos(pos, objectIn);
    // pivot.setVoltage(1);
}
    
public void FloorPickup(){
    pos = Constants.SetPoints.FIArmPivot;
}
public void PlatformPickup(){
    pos = Constants.SetPoints.PIArmPivot;
}
public void Moving(){
    pos = Constants.SetPoints.MArmPivot;
}
public void LowPlace(){
    pos = Constants.SetPoints.GSArmPivot;
}
public void MidPlace(){
    pos = Constants.SetPoints.MCOArmPivot;
}
public void HighPlace(){
    pos = Constants.SetPoints.TCOArmPivot;
}
    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}
