package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystem.PneumaticSubsystem;
import frc.robot.Constants;


public class PneumaticsRetractCommand extends CommandBase{
   private final PneumaticSubsystem pneu;


   public PneumaticsRetractCommand(PneumaticSubsystem pneu) {
       this.pneu = pneu;
       addRequirements(pneu);
   }


   @Override
   public void initialize() {
       pneu.RetractPiston();
   }
  
   @Override
   public void execute() {
   }


   @Override
   public void end(boolean interrupted) {}


   @Override
   public boolean isFinished() {
       return false;
   }
}
