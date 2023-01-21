package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Subsystem.ArmPivotSubsystem;

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
        // TODO Auto-generated method stu
        pivot.stop();
    }
    @Override
    public void execute() {
        double speed = oi.getAxis(0, Constants.Axes.LEFT_STICK_Y);
        pivot.set(speed);
    }
    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}
