package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants;
import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final CANSparkMax armPivotMotorL = new CANSparkMax(Constants.IDs.ARM_PIVOT_L, MotorType.kBrushless);
    public final CANSparkMax armPivotMotorR= new CANSparkMax(Constants.IDs.ARM_PIVOT_R, MotorType.kBrushless);
    
    private RelativeEncoder encoder = armPivotMotorR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,42);
    // spark
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
        // armPivotMotorL.
        // pid = new PIDController(0, 0, 0);
        
        armPivotMotorL.clearFaults();
        armPivotMotorR.clearFaults();
        armPivotMotorL.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        armPivotMotorL.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        armPivotMotorL.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
        // armPivotMotorL.restoreFactoryDefaults();
        // armPivotMotorR.restoreFactoryDefaults();
       
        // armPivotMotorR.follow(armPivotMotorL, true);
        
        armPivotMotorL.setSmartCurrentLimit(36);
        armPivotMotorR.setSmartCurrentLimit(36);
        
        armPivotMotorL.enableSoftLimit(SoftLimitDirection.kForward , true);
        armPivotMotorL.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armPivotMotorL.setSoftLimit(SoftLimitDirection.kForward, 20);
        armPivotMotorL.setSoftLimit(SoftLimitDirection.kReverse, -36);

        armPivotMotorR.enableSoftLimit(SoftLimitDirection.kForward , true);
        armPivotMotorR.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armPivotMotorR.setSoftLimit(SoftLimitDirection.kForward, 20);
        armPivotMotorR.setSoftLimit(SoftLimitDirection.kReverse, -36);
        pos = 0;
        armPivotMotorL.burnFlash();
        armPivotMotorR.burnFlash();
        // SparkMaxRelativeEncoder encoder = ;
       
        // encoder = armPivotMotorR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        // armPivotMotorR.getEnc
        // encoder.setInverted(true);
        
        encoder.setPosition(0);
       
        encoderRatio = 10000;
        kp =0.017;
        ki = 0.002;
        kd = 0;
        DownEmptyPID = new PIDController(kp, ki, kd);
        DownEmptyPID.setTolerance(5);
        
        DownFilledPID = new PIDController(0.022, 0.006, kd);
        DownFilledPID.setTolerance(5);
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

        double voltage = DownFilledPID.calculate(getEncoderPos(), m_setpoint.position);
        setVoltage(voltage);
        }
        else{
            pos=  setpoint;
            m_setpoint = new TrapezoidProfile.State(getEncoderPos(), 0);
            m_goal = new TrapezoidProfile.State(setpoint,0);
            var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
            m_setpoint = profile.calculate(2.5);
    
            double voltage = DownEmptyPID.calculate(getEncoderPos(), m_setpoint.position);

            setVoltage(voltage);
        }
        // }else{
        //     setVoltage(UpEmptyPID.calculate(getEncoderPos(), setpoint));
        // }
        
    }

    public double getEncoderPos(){
        // return 0;
        return -(encoder.getPosition()/encoder.getCountsPerRevolution())*encoderRatio;
    }

    public void resetEncoder(){
        encoder.setPosition(0);
    }
    // public double getRawEncoderPos(){
    //     return encoder.getPosition();
    // }
    public void stop(){
        armPivotMotorL.stopMotor();
        armPivotMotorR.stopMotor();
    }

    public void setSpeed(double speed){
        armPivotMotorL.set(speed);
        armPivotMotorR.set(-speed);

        // SmartDashboard.putNumber(getName(), speed)
    }

    public void setVoltage(double volt){
        // pos = volt;
        armPivotMotorL.set(volt);
        armPivotMotorR.set(-volt);
    }
}