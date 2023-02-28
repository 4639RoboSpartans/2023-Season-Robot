package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;

public class ElevatorSubsystem  extends SubsystemBase{
    private final WPI_TalonFX motorLeft;
    private final WPI_TalonFX motorRight;

    private final StatorCurrentLimitConfiguration StatorCurrentLimit;
    private final SupplyCurrentLimitConfiguration SupplyCurrentLimit;

    private final PIDController PID;
    private final double kp;
    private final double ki;
    private final double kd;

    private final double encoderRatio;
    private double set ;
    public static final List<WPI_TalonFX> motors = new ArrayList<>();

    public ElevatorSubsystem() {
        motorLeft = new WPI_TalonFX(Constants.IDs.ELEVATOR_MOTOR_LEFT);
        motorRight = new WPI_TalonFX(Constants.IDs.ELEVATOR_MOTOR_RIGHT);

        motorLeft.configFactoryDefault();
        motorRight.configFactoryDefault();

        motorRight.follow(motorLeft);
        motorRight.setInverted(true);
        //stator controls acceleration, current controls overall limit
        //fine if stator is above current

        StatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 30, 25, 0.01);
        SupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 30, 25, 0.01);

        motorLeft.configStatorCurrentLimit(StatorCurrentLimit);
        motorLeft.configSupplyCurrentLimit(SupplyCurrentLimit);
        motorRight.configStatorCurrentLimit(StatorCurrentLimit);
        motorRight.configSupplyCurrentLimit(SupplyCurrentLimit);


        motorLeft.configForwardSoftLimitEnable(true, 0);
        motorLeft.configReverseSoftLimitEnable(true, 0);
        motorLeft.configForwardSoftLimitThreshold(85455, 0);
        motorLeft.configReverseSoftLimitThreshold(0, 0);


                //set max limit to 11, soft limit to 10
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorRight.setNeutralMode(NeutralMode.Brake);
        

        

        getEncoderMotor().configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        encoderRatio = 66/4.6;

        kp =0.23;
        ki =0.03;
        kd=0;
        PID = new PIDController(kp, ki,kd);

        set= 0;
        motors.add(motorLeft);
        motors.add(motorRight);
    }


    public double getVoltage(){
        return  PID.calculate(getEncoderPos(), set);
       }
    public void setMotorPos(double setpoint){
        set = setpoint;
        setVoltage(PID.calculate(getEncoderPos(), setpoint));
    }

    private WPI_TalonFX getEncoderMotor(){
        return motorLeft;
    }

    public double getEncoderPos() {
        return ((getEncoderMotor().getSelectedSensorPosition())/2048)/9*encoderRatio;
    }
    public double getRawEncoderPos(){
        return getEncoderMotor().getSelectedSensorPosition();
    }
    public void setSpeed(double speed) {
         motorLeft.set(speed);
        //  motorRight.set(-speed);
    }

    public void setVoltage(double volt){
        motorLeft.setVoltage(volt);
        // motorRight.setVoltage(-volt);
    }
    // @Override
    // public void periodic() {
    //     double rawVoltage = pid.calculate(getEncoderPos());
    //     // Artificially cap the voltage
    //     double voltage = Math.signum(rawVoltage) * Math.min(Math.abs(rawVoltage), 0.15);
    //     SmartDashboard.putNumber("elevator position", getEncoderPos());
    //     // setSpeed(voltage);
    //    stop();
    //     SmartDashboard.putNumber("elevator voltage", voltage);
    //     SmartDashboard.putNumber("elevator setpoint", pid.getSetpoint());
    // }

    public void stop(){
        motorLeft.setVoltage(0);
        motorRight.setVoltage(0);
    }
}
