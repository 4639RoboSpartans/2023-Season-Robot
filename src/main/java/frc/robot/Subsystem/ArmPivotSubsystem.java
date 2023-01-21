package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final WPI_VictorSPX pivotMotor;

    public ArmPivotSubsystem(){
        pivotMotor = new WPI_VictorSPX(2);
        pivotMotor.configFactoryDefault();
        pivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void stop(){
        pivotMotor.stopMotor();
    }

    public void set(double speed){
        pivotMotor.set(speed);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}