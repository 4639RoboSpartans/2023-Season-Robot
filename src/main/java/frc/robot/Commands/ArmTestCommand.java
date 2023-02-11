package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.Constants;

public class ArmTestCommand extends CommandBase{
    private final ArmPivotSubsystem armPivot;
    private final OI oi;
    public ArmTestCommand(ArmPivotSubsystem arm, OI oi){
        armPivot = arm;
        this.oi = oi;
        addRequirements(armPivot);
    }

    @Override
    public void execute() {
        double speed =  oi.getAxis(0, Constants.Axes.RIGHT_STICK_Y);
        armPivot.set(speed * 0.05);
        super.execute();
    }
}
