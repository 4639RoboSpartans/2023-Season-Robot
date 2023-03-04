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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final CANSparkMax armPivotMotorL;
    private final CANSparkMax armPivotMotorR;
    private final RelativeEncoder encoder;
    private final double encoderRatio;

    private final PIDController DownEmptyPID;
    private final PIDController DownFilledPID;
    // private final PIDController UpFilledPID;
    private final double kp;
    private final double ki;
    private final double kd;
    private  double pos;
    private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(7, 9.5);

    private final TrapezoidProfile.Constraints mF_constraints =
    new TrapezoidProfile.Constraints(4, 4);
private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    public ArmPivotSubsystem(){
        // pid = new PIDController(0, 0, 0);
        armPivotMotorL = new CANSparkMax(Constants.IDs.ARM_PIVOT_L, MotorType.kBrushless);
        armPivotMotorR = new CANSparkMax(Constants.IDs.ARM_PIVOT_R, MotorType.kBrushless);
        armPivotMotorL.clearFaults();
        armPivotMotorR.clearFaults();
        armPivotMotorR.follow(armPivotMotorL, true);
        
        armPivotMotorL.setSmartCurrentLimit(35);
        
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
        kp =0.017;
        ki = 0.01;
        kd = 0;
        DownEmptyPID = new PIDController(kp, ki, kd);
        DownEmptyPID.setTolerance(0.1);

        DownFilledPID = new PIDController(0.024, ki, kd);
        DownFilledPID.setTolerance(0.1);
        // UpFilledPID = new PIDController(0.02, ki, kd);
        // UpFilledPID.setTolerance(0.1);

        

    }
    public double getVoltage(){
        // return pos;
        // if(pos<getEncoderPos()){
        if(Constants.objectIn){
            m_setpoint = new TrapezoidProfile.State(getEncoderPos(), 0);
            m_goal = new TrapezoidProfile.State(pos,0);
            var profile = new TrapezoidProfile(mF_constraints, m_goal, m_setpoint);
            m_setpoint = profile.calculate(2.5);
            return DownFilledPID.calculate(getEncoderPos(), m_setpoint.position);
        }else{
            m_setpoint = new TrapezoidProfile.State(getEncoderPos(), 0);
            m_goal = new TrapezoidProfile.State(pos,0);
            var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
            m_setpoint = profile.calculate(2.5);
            return DownEmptyPID.calculate(getEncoderPos(), m_setpoint.position);
        }
        
        // return UpEmptyPID.calculate(getEncoderPos(), pos);
        // }else{
        //     return UpEmptyPID.calculate(getEncoderPos(), pos);
        // }
    }
    public void setMotorPos(double setpoint, boolean coneIn){
        // if(setpoint<getEncoderPos()){
        if(coneIn){
        pos=  setpoint;
        m_setpoint = new TrapezoidProfile.State(getEncoderPos(), 0);
        m_goal = new TrapezoidProfile.State(setpoint,0);
        var profile = new TrapezoidProfile(mF_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(2.5);

        setVoltage(DownFilledPID.calculate(getEncoderPos(), m_setpoint.position));
        }
        else{
            pos=  setpoint;
            m_setpoint = new TrapezoidProfile.State(getEncoderPos(), 0);
            m_goal = new TrapezoidProfile.State(setpoint,0);
            var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
            m_setpoint = profile.calculate(2.5);
    
            setVoltage(DownEmptyPID.calculate(getEncoderPos(), m_setpoint.position));
        }
        // }else{
        //     setVoltage(UpEmptyPID.calculate(getEncoderPos(), setpoint));
        // }
        
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