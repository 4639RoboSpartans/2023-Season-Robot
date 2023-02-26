package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ArmPivotSubsystem;

public class ArmCommand extends CommandBase {
    private final OI oi;
    private final ArmPivotSubsystem pivot;
    
    public ArmCommand(ArmPivotSubsystem pivot, OI oi){
        this.pivot= pivot;
        this.oi = oi;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.stop();
    }

    @Override
    public void execute() {
        // pivot.setVoltage(6*oi.getAxis(1, Constants.Axes.RIGHT_STICK_Y));
        pivot.setMotorPos(-10);
    }
    
    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}
