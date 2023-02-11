package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.math.math;
import frc.robot.util.swerve.SwerveMovement;
public class SwerveDriveSubsystem extends SubsystemBase{
    private final SwerveModule
        moduleFrontLeft,
        moduleFrontRight,
        moduleBackLeft,
        moduleBackRight;

    public SwerveDriveSubsystem(){
        moduleFrontLeft  = new SwerveModule(Constants.IDs.MODULE_FRONT_LEFT);
        moduleFrontRight = new SwerveModule(Constants.IDs.MODULE_FRONT_RIGHT);
        moduleBackLeft   = new SwerveModule(Constants.IDs.MODULE_BACK_LEFT);
        moduleBackRight  = new SwerveModule(Constants.IDs.MODULE_BACK_RIGHT);
    }

    public void setMovement(SwerveMovement swerveMovement){

        double B = swerveMovement.forwardMovement() + swerveMovement.rotation();
        double A = swerveMovement.forwardMovement() - swerveMovement.rotation();
        double C = swerveMovement.strideMovement()  + swerveMovement.rotation();
        double D = swerveMovement.strideMovement()  - swerveMovement.rotation();

        double speedFrontLeft  = math.magnitude(A, C);
        double speedFrontRight = math.magnitude(B, C);
        double speedBackLeft   = math.magnitude(A, D);
        double speedBackRight  = math.magnitude(B, D);

        double angleFrontLeft  = math.atan2(C, A);
        double angleFrontRight = math.atan2(C, B);
        double angleBackLeft   = math.atan2(D, A);
        double angleBackRight  = math.atan2(D, B);

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

    @Override
    public void periodic() {
        moduleFrontLeft.periodic();
        moduleFrontRight.periodic();
        moduleBackLeft.periodic();
        moduleBackRight.periodic();
    }
}
