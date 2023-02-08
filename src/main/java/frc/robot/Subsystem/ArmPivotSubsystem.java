package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    private final WPI_VictorSPX armLeftMotor;
    private final WPI_VictorSPX armRightMotor;

    // private final PIDController pid; 
    // private final CANCoder canCoder;

    public ArmPivotSubsystem(){

        // pid = new PIDController(0, 0, 0);

        armLeftMotor = new WPI_VictorSPX(Constants.IDs.ARM_PIVOT_1);
        armLeftMotor.configFactoryDefault();
        armLeftMotor.setNeutralMode(NeutralMode.Brake);

        armRightMotor = new WPI_VictorSPX(Constants.IDs.ARM_PIVOT_2);
        armRightMotor.configFactoryDefault();
        armRightMotor.setNeutralMode(NeutralMode.Brake);

        // canCoder = new CANCoder(0);
    }

    public void stop(){
        armLeftMotor.stopMotor();
        armRightMotor.stopMotor();
    }

    public void set(double speed){
        armLeftMotor.set(speed);
        armRightMotor.set(-speed);
    }

    public void setAngle(double angle){
        // pid.setSetpoint(angle);
        // pid.setSetpoint(angle);
    }


    @Override
    public void periodic( ) {
    //    double speed =  pid.calculate(canCoder.getAbsolutePosition());
    //    set(speed);
    }
}