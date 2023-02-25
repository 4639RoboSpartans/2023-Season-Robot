package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase{
    private final CANSparkMax neo;
    private RelativeEncoder encoder;
    public WristSubsystem(){
       neo = new CANSparkMax(Constants.IDs.WRIST_MOTOR, MotorType.kBrushless);
      encoder = neo.getEncoder();
    }
    public double getEncoder(){
      return encoder.getPosition();
    }
    public void setSpeed(double speed){
      neo.set(speed);
    }

    public void stop(){
        neo.set(0);
    }
}
//hi