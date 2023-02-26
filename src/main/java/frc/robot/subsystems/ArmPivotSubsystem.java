package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final CANSparkMax armPivotMotorL;
    private final CANSparkMax armPivotMotorR;
    private final RelativeEncoder encoder;
    private final double encoderRatio;

    private final PIDController PID;
    private final double kp;
    private final double ki;
    private final double kd;
    private  double pos;
    public ArmPivotSubsystem(){
        // pid = new PIDController(0, 0, 0);
        armPivotMotorL = new CANSparkMax(Constants.IDs.ARM_PIVOT_L, MotorType.kBrushless);
        armPivotMotorR = new CANSparkMax(Constants.IDs.ARM_PIVOT_R, MotorType.kBrushless);
        armPivotMotorL.clearFaults();
        armPivotMotorR.clearFaults();
        armPivotMotorR.follow(armPivotMotorL, true);
        
        armPivotMotorL.setSmartCurrentLimit(5);
        
        // armPivotMotorL.enableSoftLimit(SoftLimitDirection.kForward , true);
        armPivotMotorL.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // armPivotMotorL.setSoftLimit(SoftLimitDirection.kForward, 0);
        armPivotMotorL.setSoftLimit(SoftLimitDirection.kReverse, -36);
        pos = 0;

        encoder = armPivotMotorR.getEncoder();
        
        encoder.setPosition(0);
        armPivotMotorL.burnFlash();
        armPivotMotorR.burnFlash();
        encoderRatio = 10000;
        kp =0.02;
        ki = 0;
        kd = 0;
        PID = new PIDController(kp, ki, kd);

    }
    public double getVoltage(){
        // return pos;
        return PID.calculate(getEncoderPos(), pos);
    }
    public void setMotorPos(double setpoint){
        pos=  setpoint;
        setVoltage(PID.calculate(getEncoderPos(), setpoint));
    }

    public double getEncoderPos(){
        return (encoder.getPosition()/encoder.getCountsPerRevolution())*encoderRatio;
    }

    public double getRawEncoderPos(){
        return encoder.getPosition();
    }
    public void stop(){
        armPivotMotorL.stopMotor();
        armPivotMotorR.stopMotor();
    }

    public void setSpeed(double speed){
        armPivotMotorL.set(speed);
        armPivotMotorR.set(-speed);
    }

    public void setVoltage(double volt){
        // pos = volt;
        armPivotMotorL.set(volt);
        armPivotMotorR.set(-volt);
    }
}