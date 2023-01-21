package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystem.PneumaticSubsystem;
import frc.robot.Constants;


public class PneumaticsExtendCommand extends CommandBase{
   private final PneumaticSubsystem pneu;


   public PneumaticsExtendCommand(PneumaticSubsystem pneu) {
       this.pneu = pneu;
       addRequirements(pneu);
       System.out.println("piston working ur cool");
   }


   @Override
   public void initialize() {
       pneu.extendPiston();
   }


   @Override
   public void execute() {
   }


   @Override
   public void end(boolean interrupted) {
   }


   @Override
   public boolean isFinished() {
       return false;
   }
}
