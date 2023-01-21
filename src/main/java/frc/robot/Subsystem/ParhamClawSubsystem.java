package frc.robot.Subsystem;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ParhamClawSubsystem extends SubsystemBase{
   private final Solenoid m_solenoid;
   private final WPI_VictorSPX motor1;
   private final WPI_VictorSPX motor2;

   public ParhamClawSubsystem(){
       m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
       motor1 = new WPI_VictorSPX(0);
       motor2 = new WPI_VictorSPX(1);
       motor1.configFactoryDefault();
       motor2.configFactoryDefault();
       motor1.setNeutralMode(NeutralMode.Brake);
       motor2.setNeutralMode(NeutralMode.Brake);
   }
   public void extendPiston() {
       m_solenoid.set(true);
       motor1.set(1);
       motor2.set(-1);
   }
   public void RetractPiston() {
       m_solenoid.set(false);
       motor1.set(-1);
       motor2.set(1);
   }

   public void stop(){
        motor1.set(0);
        motor2.set(0);
   }
}