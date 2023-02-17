package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotInfo.DriveConstants;
import frc.robot.commands.navXCommand;
import frc.robot.math.math;
import frc.robot.swerve.SwerveMovement;
public class SwerveDriveSubsystem extends SubsystemBase{
    private final SwerveModule
        moduleFrontLeft,
        moduleFrontRight,
        moduleBackLeft,
        moduleBackRight;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final NavX navX;
    private final double d;

    private Pose2d pose;
    
    public SwerveDriveSubsystem(){
        moduleFrontLeft  = new SwerveModule(Constants.IDs.MODULE_FRONT_LEFT);
        moduleFrontRight = new SwerveModule(Constants.IDs.MODULE_FRONT_RIGHT);
        moduleBackLeft   = new SwerveModule(Constants.IDs.MODULE_BACK_LEFT);
        moduleBackRight  = new SwerveModule(Constants.IDs.MODULE_BACK_RIGHT);

        navX = new NavX();
        pose = new Pose2d();

        d = Constants.RobotInfo.robotBaseLength / 2;
       kinematics = Constants.RobotInfo.DriveConstants.kDriveKinematics;
        // Creating my odometry object from the kinematics object and the initial wheel positions.
        // Here, our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing the opposing alliance wall.
        odometry = new SwerveDriveOdometry(
            kinematics, navX.getGyroRotation2d(),
            getSwervemodulePositions(),
            new Pose2d(5.0, 13.5, new Rotation2d())
        );
    }

    public void setMovement(SwerveMovement swerveMovement){
        double A = swerveMovement.forwardMovement() - swerveMovement.rotation();
        double B = swerveMovement.forwardMovement() + swerveMovement.rotation();
        double C = swerveMovement.strideMovement()  + swerveMovement.rotation();
        double D = swerveMovement.strideMovement()  - swerveMovement.rotation();

        double speedFrontLeft  = math.magnitude(A, C);
        double speedFrontRight = math.magnitude(B, C);
        double speedBackLeft   = math.magnitude(A, D);
        double speedBackRight  = math.magnitude(B, D);

        double angleFrontLeft  = math.atan(C, A);
        double angleFrontRight = math.atan(C, B);
        double angleBackLeft   = math.atan(D, A);
        double angleBackRight  = math.atan(D, B);

        double max = math.max(
            speedFrontLeft, 
            speedFrontRight, 
            speedBackLeft, 
            speedBackRight
        );

        if (max > 1) {
            speedFrontLeft  /= max;
            speedFrontRight /= max;
            speedBackLeft   /= max;
            speedBackRight  /= max;
        }

        SmartDashboard.putNumber("angle 1", angleFrontLeft);
        SmartDashboard.putNumber("angle 2", angleFrontRight);
        SmartDashboard.putNumber("angle 3", angleBackLeft);
        SmartDashboard.putNumber("angle 4", angleBackRight);

        setModules(
            new SwerveModuleState(speedFrontLeft, Rotation2d.fromDegrees(angleFrontLeft)),
            new SwerveModuleState(speedFrontRight, Rotation2d.fromDegrees(angleFrontRight)),
            new SwerveModuleState(speedBackLeft, Rotation2d.fromDegrees(angleBackLeft)),
            new SwerveModuleState(speedBackRight, Rotation2d.fromDegrees(angleBackRight))
        );
    }

    public void setModules(SwerveModuleState[] states){
        setModules(states[0], states[1], states[2], states[3]);
    }
   

    public void setModules(
            SwerveModuleState stateFrontLeft,
            SwerveModuleState stateFrontRight,
            SwerveModuleState stateBackLeft,
            SwerveModuleState stateBackRight
    ){
        moduleFrontLeft .setState(stateFrontLeft);
        moduleFrontRight.setState(stateFrontRight);
        moduleBackLeft  .setState(stateBackLeft);
        moduleBackRight .setState(stateBackRight);
    }
    
    public void setModules(SwerveModuleState state){
        setModules(state, state, state, state);
    }

    public void stop(){
        setModules(new SwerveModuleState());
    }

    public void resetPose() {
        odometry.resetPosition(
        navX.getGyroRotation2d(),
        getSwervemodulePositions(), 
        new Pose2d(5.0, 13.5, new Rotation2d()));
    }

    public SwerveModulePosition[] getSwervemodulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(d * Math.sqrt(2), Rotation2d.fromDegrees(135)),
            new SwerveModulePosition(d * Math.sqrt(2), Rotation2d.fromDegrees(45)),
            new SwerveModulePosition(d * Math.sqrt(2), Rotation2d.fromDegrees(-135)),
            new SwerveModulePosition(d * Math.sqrt(2), Rotation2d.fromDegrees(-45))
        };
    }

    public Rotation2d getRotation() {
        return navX.getGyroRotation2d();
    }

    @Override
    public void periodic() {
        moduleFrontLeft.periodic();
        moduleFrontRight.periodic();
        moduleBackLeft.periodic();
        moduleBackRight.periodic();

          // Get the rotation of the robot from the gyro.
        var gyroAngle = navX.getGyroRotation2d();

        // Update the pose
        pose = odometry.update(gyroAngle, getSwervemodulePositions());
    }


    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose2d) {
        //rotation2d reset may be wrong
        odometry.resetPosition(
          new Rotation2d(),
          getSwervemodulePositions(),
          getPose()
        );

    }
}
