package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase{
    private final CANSparkMax neo;
    private RelativeEncoder encoder;
    private final double encoderRatio;
    public WristSubsystem(){
       neo = new CANSparkMax(Constants.IDs.WRIST_MOTOR, MotorType.kBrushless);
      encoder = neo.getEncoder();
      encoderRatio = 45;
    }
    //neo has 42 countes per rev
    public double getEncoderPos(){
      return (encoder.getPosition()/encoder.getCountsPerRevolution())*encoderRatio;
    }
    public void setSpeed(double speed){
      neo.set(speed);
    }

    public void stop(){
        neo.set(0);
    }
}
//hi