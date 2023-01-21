package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystem.ParhamClawSubsystem;
import frc.robot.Subsystem.PneumaticSubsystem;
import frc.robot.Constants;


public class ParhamRetractCommand extends CommandBase{
   private final ParhamClawSubsystem pneu;


   public ParhamRetractCommand(ParhamClawSubsystem pneu) {
       this.pneu = pneu;
       addRequirements(pneu);
       System.out.println("piston working ur cool");
   }


   @Override
   public void initialize() {
       pneu.RetractPiston();
   }


   @Override
   public void execute() {
   }


   @Override
   public void end(boolean interrupted) {
        pneu.stop();
   }


   @Override
   public boolean isFinished() {
       return false;
   }
}
