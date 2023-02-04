package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final WPI_VictorSPX armPivotMotor1;
    private final WPI_VictorSPX armPivotMotor2;

    private final PIDController pid; 
    private final CANCoder canCoder;

    public ArmPivotSubsystem(){

        pid = new PIDController(0, 0, 0);

        armPivotMotor1 = new WPI_VictorSPX(Constants.IDs.ARM_PIVOT_1);
        armPivotMotor1.configFactoryDefault();
        armPivotMotor1.setNeutralMode(NeutralMode.Brake);

        armPivotMotor2 = new WPI_VictorSPX(Constants.IDs.ARM_PIVOT_2);
        armPivotMotor2.configFactoryDefault();
        armPivotMotor2.setNeutralMode(NeutralMode.Brake);

        canCoder = new CANCoder(0);
    }

    public void stop(){
        armPivotMotor1.stopMotor();
        armPivotMotor2.stopMotor();
    }

    public void set(double speed){
        armPivotMotor1.set(speed);
        armPivotMotor2.set(-speed);
    }

    public void setAngle(double angle){
        pid.setSetpoint(angle);
        pid.setSetpoint(angle);
    }


    @Override
    public void periodic( ) {
       double speed =  pid.calculate(canCoder.getAbsolutePosition());
       set(speed);
    }
}