package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final CANSparkMax armPivotMotorL;
    private final CANSparkMax armPivotMotorR;
    private final RelativeEncoder encoder;
    // private final PIDController pid; 
    // private final CANCoder canCoder;

    public ArmPivotSubsystem(){
        // pid = new PIDController(0, 0, 0);
        armPivotMotorL = new CANSparkMax(Constants.IDs.ARM_PIVOT_L, MotorType.kBrushless);
        armPivotMotorR = new CANSparkMax(Constants.IDs.ARM_PIVOT_R, MotorType.kBrushless);
        encoder = armPivotMotorL.getEncoder();
        // canCoder = new CANCoder(Constants.IDs.ENCODER_3);
    }
    public double getEncoder(){
        return encoder.getPosition();
    }
    public void stop(){
        armPivotMotorL.stopMotor();
        armPivotMotorR.stopMotor();
    }

    public void set(double speed){
        armPivotMotorL.set(speed);
        armPivotMotorR.set(-speed);
    }

    // public void setAngle(double angle){
    //     pid.setSetpoint(angle);
    //     pid.setSetpoint(angle);
    // }

    @Override
    public void periodic( ) {
    //    double speed =  pid.calculate(canCoder.getPosition());
    //    set(speed);
    }
}