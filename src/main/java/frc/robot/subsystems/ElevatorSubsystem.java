package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        pid = new PIDController(1.2, 0.09, 0);
        pid.setSetpoint(0.2);
    }

    private WPI_TalonFX getEncoderMotor(){
        return motorLeft;
    }

    private double getPosition() {
        return getEncoderMotor().getSelectedSensorPosition() * .00001;
    }

    public void setPosition(double position) {
        pid.setSetpoint(position);
    }

    private void setSpeed(double speed) {
         motorLeft.set(speed);
         motorRight.set(-speed);
    }

    @Override
    public void periodic() {
        double voltage = pid.calculate(getPosition());
        SmartDashboard.putNumber("Encoder Position", getPosition());
    //    setSpeed(voltage);
        stop();
        SmartDashboard.putNumber("elevator voltage", voltage);
        SmartDashboard.putNumber("pid diff", pid.getSetpoint() - getPosition());
    }

    public void stop(){
        motorLeft.set(0);
        motorRight.set(0);
    }
}
