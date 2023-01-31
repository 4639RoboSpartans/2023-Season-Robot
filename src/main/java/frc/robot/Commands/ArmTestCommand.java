package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystem.ArmPivotSubsystem;
import frc.robot.Constants;

public class ArmTestCommand extends CommandBase{
    private final ArmPivotSubsystem ArmPivot;
    private final OI oi;
    public ArmTestCommand(ArmPivotSubsystem arm, OI oi){
        ArmPivot = arm;
        this.oi = oi;
        addRequirements(ArmPivot);
    }

    @Override
    public void execute() {
        double position =  oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        ArmPivot.set(position);
        super.execute();
    }
}
