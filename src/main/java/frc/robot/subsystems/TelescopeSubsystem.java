package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopeSubsystem  extends SubsystemBase{

    public WPI_TalonSRX motor;
    // private final CANCoder encoder;
    private final double encoderRatio;
    private final PIDController PID;
    private final double kp;
    private final double ki;
    private final double kd;
    private final SupplyCurrentLimitConfiguration SupplyCurrentLimit;
    private final double softLimit;
    double pos=0;
    public TelescopeSubsystem() {
        motor = new WPI_TalonSRX(Constants.IDs.TELESCOPE_MOTOR);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        encoderRatio = 30/4.315;

        SupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 11, 10, 0.01);
        motor.configSupplyCurrentLimit(SupplyCurrentLimit);
        
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);
        softLimit = -16770;
        motor.configForwardSoftLimitThreshold(0, 0);
        motor.configReverseSoftLimitThreshold(softLimit, 0);

        kp =0.37;
        ki = 0.1;
        kd = 0;
        PID = new PIDController(kp, ki, kd);
        PID.setTolerance(0.1);
    }

    public double getVoltage(){
        return PID.calculate(getEncoderPos(),pos);
    }
    public void setMotorPos(double setpoint) {
        pos = setpoint;
        setVoltage(PID.calculate(getEncoderPos(),setpoint));
    }

    public double getEncoderPos(){
        return (motor.getSelectedSensorPosition()/4096)*encoderRatio;
    }

    public double getRawEncoderPos(){
        return motor.getSelectedSensorPosition();
    }
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void setVoltage(double volt){
        motor.setVoltage(volt);
    }
    // @Override
    // public void periodic() {
    //     double voltage = pid.calculate(encoder.getPosition());
    //     motor.set(voltage);
    // }
}
