package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConfig;
import frc.robot.math.math;
import frc.robot.swerve.SwerveUtil;

public class SwerveModule {
    private final WPI_TalonFX driver, rotator;
    private final CANCoder rotationEncoder;
    private final PIDController rotationPID;

    private final double rotationOffset;
    
    private final double kp = 0.005;
    private final double ki = 0;
    private final double kd = 0;

    private double speed = 0;

    private final double DriveConversionFactor;

    private StatorCurrentLimitConfiguration StatorCurrentLimit;
    private StatorCurrentLimitConfiguration StatorCurrentLimitR;
    public SwerveModule(SwerveModuleConfig swerveModuleData){
        DriveConversionFactor = (1/2048)*(1/6.55)*(0.1016)*Math.PI;

        driver = new WPI_TalonFX(swerveModuleData.driveMotorID());
        rotator = new WPI_TalonFX(swerveModuleData.rotaterMotorID());

        driver.configFactoryDefault();
        rotator.configFactoryDefault();

        driver.setNeutralMode(NeutralMode.Coast);
        driver.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rotator.setNeutralMode(NeutralMode.Coast); //24 23  and 26 25
        StatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 26, 25, 0.01);
        StatorCurrentLimitR = new StatorCurrentLimitConfiguration(true, 26, 25, 0.01);
        driver.configStatorCurrentLimit(StatorCurrentLimit);
        rotator.configStatorCurrentLimit(StatorCurrentLimitR);
        

        rotationEncoder = new CANCoder(swerveModuleData.encoderID());
        rotationEncoder.configFactoryDefault();

        rotationOffset = swerveModuleData.rotationOffset();

        rotationPID = new PIDController(
            Constants.RobotInfo.ROTATOR_MOTOR_KP,
            Constants.RobotInfo.ROTATOR_MOTOR_KI,
            0
        );
        rotationPID.setTolerance(0.1);
        rotationPID.enableContinuousInput(-180, 180);

    }

    public void ClimbingMode(){
        StatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 33, 32, 0.01);
        driver.configStatorCurrentLimit(StatorCurrentLimit);


    }
    public void DriveMode(){
        StatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 24, 23, 0.01);
        driver.configStatorCurrentLimit(StatorCurrentLimit);
    }

    public void periodic() {
        double rotate = rotationPID.calculate(getRotationInDegrees());
        rotator.setVoltage(-rotationPID.calculate(getRotationInDegrees()));
        driver.set(speed * Constants.RobotInfo.MOVEMENT_SPEED);

        // SmartDashboard.putNumber("Module Speed " + driver.getDeviceID(), speed);
        // SmartDashboard.putNumber("Module Rotation Encoder " + driver.getDeviceID(), getRotationInDegrees());
        // SmartDashboard.putNumber("Module Rotation Setpoint " + driver.getDeviceID(), rotationPID.getSetpoint());
        // SmartDashboard.putNumber("Module Rotation Voltage"+rotator.getDeviceID(),    -rotationPID.calculate(getRotationInDegrees()));
        // SmartDashboard.putNumber("Module Rotation Value " + driver.getDeviceID(), rotate);
    }

    public double getRotationInDegrees(){
        double rawRotation = rotationEncoder.getAbsolutePosition() - rotationOffset;
        // Constrain angle to be between -180 and 180
        return math.mod(rawRotation, -180, 180);
    }

    private void setSpeed(double speed){
        this.speed = speed;
    }

    private void setRotation(double degrees){
        rotationPID.setSetpoint(degrees);
    }

    public double getDriveVelocity(){
        return driver.getSelectedSensorVelocity()*DriveConversionFactor;
    }
    public double getDriveDistance(){
        return driver.getSelectedSensorPosition()*DriveConversionFactor;
    }

    // public double getDriveDistance(){
    //     return ((driver.getSelectedSensorPosition()/4096)/6.55)*(0.1*Math.PI);
    // }
    // public double getRadRotation(){
    //     return Math.toRadians(rotationEncoder.getPosition());
    // }
    // public SwerveModulePosition getPosition(){
    //     return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getRadRotation()));
    // }

    public double getTurningVelocity(){
        return rotationEncoder.getVelocity();
    }
    public double getTurningPosition(){
        double rawRotation = rotationEncoder.getAbsolutePosition() - rotationOffset;
        // Constrain angle to be between -180 and 180
        return rawRotation;
    }

    public SwerveModuleState getState() {
        double moduleSpeed = getDriveVelocity();
        double rotationDegrees = getRotationInDegrees();
        Rotation2d rotation = Rotation2d.fromDegrees(rotationDegrees);
        return new SwerveModuleState(moduleSpeed, rotation);
    }

    public void setState(SwerveModuleState state){
        if(isNegligible(state)) stop();
        else{
            // SwerveModuleState optimizedState = SwerveUtil.optimize(state, getRotationInDegrees());
            SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getRotationInDegrees()));
            setSpeed(optimizedState.speedMetersPerSecond);
            setRotation(optimizedState.angle.getDegrees());
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
