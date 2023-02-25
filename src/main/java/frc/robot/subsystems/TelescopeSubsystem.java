package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopeSubsystem  extends SubsystemBase{

    public WPI_TalonSRX motor;
    // private final CANCoder encoder;
    private final PIDController pid;
    private final double encoderRatio;
    public TelescopeSubsystem() {
        motor = new WPI_TalonSRX(Constants.IDs.TELESCOPE_MOTOR);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        pid = new PIDController(0.0000000001, 0, 0);
        encoderRatio = 1;
    }

    // public void setPosition(double position) {
    //     pid.setSetpoint(position);
    // }

    public double getEncoderPos(){
        return (motor.getSelectedSensorPosition()/4096)*encoderRatio;
    }

    public void extend(double speed) {
        motor.set(speed);
    }

    // @Override
    // public void periodic() {
    //     double voltage = pid.calculate(encoder.getPosition());
    //     motor.set(voltage);
    // }
}
