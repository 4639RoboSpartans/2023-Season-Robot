package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase{
    private final CANSparkMax neo;
    private RelativeEncoder encoder;
    private final double encoderRatio;
    private final PIDController PID;
    private final double kp;
    private final double ki;
    private final double kd;
    private double set;
    public WristSubsystem(){
       neo = new CANSparkMax(Constants.IDs.WRIST_MOTOR, MotorType.kBrushless);
      encoder = neo.getEncoder();
      encoderRatio = 0.5;
      encoder.setPosition(0);
      kp = 1.3;
      ki=1;
      kd=0;
      PID = new PIDController(kp, ki, kd);
      PID.setTolerance(0.1);
      set=0;
    }
    public double getVoltage(){
     return  PID.calculate(getEncoderPos(), set);
    }
    public void setMotorPos(double setpoint){
      set = setpoint;
      setVoltage(PID.calculate(getEncoderPos(), setpoint));
      
    }

    //neo has 42 countes per rev
    public double getEncoderPos(){
      return ((encoder.getPosition()/encoder.getCountsPerRevolution())/encoderRatio)*360;
    }
    public double getCPR(){
      return encoder.getCountsPerRevolution();
    }


    public void setSpeed(double speed){
      neo.set(speed);
    }
    public void setVoltage(double volts){
      neo.setVoltage(volts);
    }

    public void stop(){
        neo.set(0);
    }
}
//hi