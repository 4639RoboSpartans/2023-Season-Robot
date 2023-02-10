package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.math.math;
import frc.robot.util.swerve.SwerveMovement;
import frc.robot.util.swerve.SwerveUtil;
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

        double A = swerveMovement.forwardMovement() + swerveMovement.rotation();
        double B = swerveMovement.forwardMovement() - swerveMovement.rotation();
        double C = swerveMovement.strideMovement()  + swerveMovement.rotation();
        double D = swerveMovement.strideMovement()  - swerveMovement.rotation();

        double speedFrontLeft  = math.magnitude(A, C);
        double speedFrontRight = math.magnitude(B, C);
        double speedBackLeft   = math.magnitude(A, D);
        double speedBackRight  = math.magnitude(B, D);

        double angleFrontLeft  = math.atan2(A, C);
        double angleFrontRight = Math.atan2(B, C);
        double angleBackLeft   = Math.atan2(A, D);
        double angleBackRight  = Math.atan2(B, D);

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

        setModules(
            new SwerveModuleState(speedFrontLeft, new Rotation2d(angleFrontLeft)),
            new SwerveModuleState(speedFrontRight, new Rotation2d(angleFrontRight)),
            new SwerveModuleState(speedBackLeft, new Rotation2d(angleBackLeft)),
            new SwerveModuleState(speedBackRight, new Rotation2d(angleBackRight))
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
}
