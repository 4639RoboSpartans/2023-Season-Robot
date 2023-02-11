package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem  extends SubsystemBase{
    private final WPI_TalonFX motorLeft;
    private final WPI_TalonFX motorRight;

    private final PIDController pid;

    public ElevatorSubsystem() {
        motorLeft = new WPI_TalonFX(Constants.IDs.ELEVATOR_MOTOR_LEFT);
        motorRight = new WPI_TalonFX(Constants.IDs.ELEVATOR_MOTOR_RIGHT);
        motorLeft.configFactoryDefault();
        motorRight.configFactoryDefault();
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorRight.setNeutralMode(NeutralMode.Brake);
        
        getEncoderMotor().configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        pid = new PIDController(0.01, 0, 0);
    }

    private WPI_TalonFX getEncoderMotor(){
        return motorLeft;
    }

    public void setPosition(double position) {
        pid.setSetpoint(position);
    }

    @Override
    public void periodic() {
        double voltage = pid.calculate(getEncoderMotor().getSelectedSensorPosition());
        // motorLeft.set(voltage);
        // motorRight.set(-voltage);
    }
}
