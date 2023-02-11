package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem  extends SubsystemBase{
    public WPI_TalonFX motorLeft;
    public WPI_TalonFX motorRight;

    private final PIDController pid;

    public ElevatorSubsystem() {
        motorLeft = new WPI_TalonFX(Constants.IDs.TELESCOPE_MOTOR_LEFT);
        motorLeft.configFactoryDefault();
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        pid = new PIDController(0.01, 0, 0);
    }

    public void setPosition(double position) {
        pid.setSetpoint(position);
    }

    @Override
    public void periodic() {
        double voltage = pid.calculate(motorLeft.getSelectedSensorPosition());
        motorLeft.set(voltage);
    }
}
