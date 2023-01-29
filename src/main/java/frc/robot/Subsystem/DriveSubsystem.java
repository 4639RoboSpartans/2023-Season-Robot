package frc.robot.Subsystem;
import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    public SwerveModule SwerveMod1FrontRight;
    public SwerveModule SwerveMod2FrontLeft;
    public SwerveModule SwerveMod3RearLeft;
    public SwerveModule SwerveMod4RearRight;

    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );
    
    SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0));
    //hello
   public AHRS navx = new AHRS();
    public SwerveDriveOdometry m_odometry;
    public DriveSubsystem(){
        SwerveMod1FrontRight = new SwerveModule(Constants.IDs.DriverMotor1, Constants.IDs.RotaterMotor1, Constants.IDs.ModEncoder1);
        SwerveMod2FrontLeft = new SwerveModule(Constants.IDs.DriverMotor2, Constants.IDs.RotaterMotor2, Constants.IDs.ModEncoder2);
        SwerveMod3RearLeft = new SwerveModule(Constants.IDs.DriverMotor3, Constants.IDs.RotaterMotor3, Constants.IDs.ModEncoder3);
        SwerveMod4RearRight = new SwerveModule(Constants.IDs.DriverMotor4, Constants.IDs.RotaterMotor4, Constants.IDs.ModEncoder4);
        navx.calibrate();
        m_odometry=new SwerveDriveOdometry(m_kinematics,
        navx.getRotation2d());
    }

    public void zeroHeading() {
        navx.reset();
    }

    public double getHeading() {
        if(navx.getAngle()%360<0){
            return (navx.getAngle()%360)+360;
        }
        return navx.getAngle()%360;
    }

    public double getHeadingRadians(){
        return Math.toRadians(getHeading());
    }

    public Rotation2d getHeadingRotation(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void stop(){
        SwerveMod1FrontRight.stop();
        SwerveMod2FrontLeft.stop();
        SwerveMod3RearLeft.stop();
        SwerveMod4RearRight.stop();
    }

    public void resetAnglesAndPositions(){
        SwerveMod1FrontRight.resetAngleAndPosition();
        SwerveMod2FrontLeft.resetAngleAndPosition();
        SwerveMod3RearLeft.resetAngleAndPosition();
        SwerveMod4RearRight.resetAngleAndPosition();
    }

    public void setModule1(double speed, double rotation){
        SwerveMod1FrontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(rotation)));
    }
    public void setModule2(double speed, double rotation){
        SwerveMod2FrontLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(rotation)));
    }
    public void setModule3(double speed, double rotation){
        SwerveMod3RearLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(rotation)));
    }
    public void setModule4(double speed, double rotation){
        SwerveMod4RearRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(rotation)));
    }

    public void setModuleStates(SwerveModuleState[] states){
        double maxSpeed = Arrays.stream(states).map(i->i.speedMetersPerSecond).reduce(0.0, (a,b)->Math.max(a,b));
        if(maxSpeed > Constants.kMaxSpeedMetersPerSecond){
            for(SwerveModuleState state : states){
                state.speedMetersPerSecond *= Constants.kMaxSpeedMetersPerSecond / maxSpeed;
            }
        }

        SwerveMod1FrontRight.setDesiredState(new SwerveModuleState(states[0].speedMetersPerSecond, Rotation2d.fromDegrees(states[0].angle.getDegrees())));
        SwerveMod2FrontLeft.setDesiredState(new SwerveModuleState(states[1].speedMetersPerSecond, Rotation2d.fromDegrees(states[1].angle.getDegrees())));
        SwerveMod3RearLeft.setDesiredState(new SwerveModuleState(states[2].speedMetersPerSecond, Rotation2d.fromDegrees(states[2].angle.getDegrees())));
        SwerveMod4RearRight.setDesiredState(new SwerveModuleState(states[3].speedMetersPerSecond, Rotation2d.fromDegrees(states[3].angle.getDegrees())));
    }

    public void resetEncoders(){
        SwerveMod1FrontRight.resetEncoder();
        SwerveMod2FrontLeft.resetEncoder();
        SwerveMod3RearLeft.resetEncoder();
        SwerveMod4RearRight.resetEncoder();
    }

    @Override
    public void periodic(){
        odometer.update(Rotation2d.fromDegrees(getHeading()), 
            SwerveMod1FrontRight.getState(), SwerveMod2FrontLeft.getState(),
            SwerveMod3RearLeft.getState(), SwerveMod4RearRight.getState()
        );
    }
}