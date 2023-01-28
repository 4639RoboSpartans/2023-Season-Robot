package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final WPI_VictorSPX armPivotMotor;
    private final PIDController pid; 
    private final CANCoder canCoder;

    public ArmPivotSubsystem(){
        pid = new PIDController(0, 0, 0);
        armPivotMotor = new WPI_VictorSPX(Constants.IDs.ARM_PIVOT_ID);
        armPivotMotor.configFactoryDefault();
        armPivotMotor.setNeutralMode(NeutralMode.Brake);
        canCoder = new CANCoder(0);
    }

    public void stop(){
        armPivotMotor.stopMotor();
    }

    public void set(double speed){
        armPivotMotor.set(speed);
    }

    public void setAngle(double angle){
        pid.setSetpoint(angle);
    }


    @Override
    public void periodic() {
       double speed =  pid.calculate(canCoder.getAbsolutePosition());
       set(speed);
    }
}