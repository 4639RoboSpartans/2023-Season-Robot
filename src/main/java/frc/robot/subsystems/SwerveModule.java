package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConfig;
import frc.robot.util.math.math;
import frc.robot.util.swerve.SwerveUtil;

public class SwerveModule {
    private final WPI_TalonFX driver, rotator;
    private final CANCoder encoder;
    private final PIDController rotationPID;

    private final double rotationOffset;

    private final double kp = 0.09;
    private final double ki = 0.15;
    private final double kd = 0;

    private double speed;

    public SwerveModule(SwerveModuleConfig swerveModuleData){
        driver = new WPI_TalonFX(swerveModuleData.driveMotorID);
        rotator = new WPI_TalonFX(swerveModuleData.rotaterMotorID);

        driver.configFactoryDefault();
        rotator.configFactoryDefault();

        driver.setNeutralMode(NeutralMode.Coast);
        rotator.setNeutralMode(NeutralMode.Coast);

        driver.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        encoder = new CANCoder(swerveModuleData.encoderID);
        encoder.configFactoryDefault();

        rotationOffset = swerveModuleData.rotationOffset;

        rotationPID = new PIDController(kp, ki, kd);
    }

    public void periodic() {
        rotator.set(-rotationPID.calculate(getRotationInDegrees()));
        driver.set(Constants.RobotInfo.MOVEMENT_SPEED * 0.2);

        SmartDashboard.putNumber("Module Speed " + driver.getDeviceID(), speed);
        SmartDashboard.putNumber("Module Rotation " + driver.getDeviceID(), rotationPID.getSetpoint());
    }

    public double getRotationInDegrees(){
        double rotation = encoder.getAbsolutePosition() - rotationOffset;
        return math.round(math.mod(rotation, -180, 180), 0.5);
    }

    private void setSpeed(double speed){
        this.speed = speed;
    }

    public void setRotation(double degrees){
        rotationPID.setSetpoint(degrees);
    }

    public double getVelocity(){
        return driver.getSelectedSensorVelocity();
    }

    public double getTurningVelocity(){
        return encoder.getVelocity();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getRotationInDegrees()));
    }

    public void setState(SwerveModuleState state){
        if(isNegligible(state)) stop();
        else{
            SwerveModuleState optimized = SwerveUtil.optimize(state, getRotationInDegrees());
            setSpeed(optimized.speedMetersPerSecond);
            setRotation(optimized.angle.getDegrees());
        }
    }

    public void resetAngleAndPosition(){
        setSpeed(0);
        setRotation(0);
    }

    public void stop(){
        setSpeed(0);
        setRotation(getRotationInDegrees());

        driver.stopMotor();
        rotator.stopMotor();
    }
    
    private static boolean isNegligible(SwerveModuleState state){
        return state.speedMetersPerSecond < 0.001;
    }
}