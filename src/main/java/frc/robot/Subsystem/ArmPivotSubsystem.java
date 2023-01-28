package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final WPI_VictorSPX pivotMotor;

    public ArmPivotSubsystem(){
        pivotMotor = new WPI_VictorSPX(Constants.IDs.ARM_PIVOT_ID);
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
        super.periodic();
    }
}