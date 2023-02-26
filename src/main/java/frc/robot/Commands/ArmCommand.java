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
    public ArmCommand(ArmPivotSubsystem pivot, OI oi){
        this.pivot= pivot;
        this.oi = oi;
        pos=0;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.stop();
    }

    @Override
    public void execute() {
        // pivot.setVoltage(6*oi.getAxis(1, Constants.Axes.RIGHT_STICK_Y));
        if(oi.getButton(1, Constants.Buttons.X_BUTTON).getAsBoolean()){
            pos = Constants.SetPoints.MArmPivot;
    }
    
    if(oi.getButton(1, Constants.Buttons.Y_BUTTON).getAsBoolean()){
            pos = Constants.SetPoints.TCOArmPivot;;
    }
    if(oi.getButton(1, Constants.Buttons.B_BUTTON).getAsBoolean()){
            pos = Constants.SetPoints.GSArmPivot;
    }
    if(oi.getButton(1, Constants.Buttons.A_BUTTON).getAsBoolean()){
        // pos=;
        pos = Constants.SetPoints.MCOArmPivot;
    }
    
    pivot.setMotorPos(pos);
}
    
    
    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}
