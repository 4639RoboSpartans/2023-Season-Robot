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
        double wa1 = 0, wa2 = 0, wa3 = 0, wa4 = 0;

        double A = swerveMovement.strideMovement()  - swerveMovement.rotation() / Math.sqrt(2);
        double B = swerveMovement.strideMovement()  + swerveMovement.rotation() / Math.sqrt(2);
        double C = swerveMovement.forwardMovement() - swerveMovement.rotation() / Math.sqrt(2);
        double D = swerveMovement.forwardMovement() + swerveMovement.rotation() / Math.sqrt(2);


        double wheelSpeedFrontLeft = math.magnitude(B, C);
        double wheelSpeedFrontRight = math.magnitude(B, D);
        double wheelSpeedBackLeft = math.magnitude(A, C);
        double wheelSpeedBackRight = math.magnitude(A, D);

        if (wheelSpeedBackLeft > 0.05 || wheelSpeedBackLeft < -0.05) {
            wa1 = Math.atan2(B, C) * 180 / Math.PI;
        }
        if (wheelSpeedBackLeft > 0.05 || wheelSpeedBackLeft < -0.05) {
            wa2 = Math.atan2(B, D) * 180 / Math.PI;
        }
        if (wheelSpeedBackLeft > 0.05 || wheelSpeedBackLeft < -0.05) {
            wa3 = Math.atan2(A, D) * 180 / Math.PI;
        }
        if (wheelSpeedBackLeft > 0.05 || wheelSpeedBackLeft < -0.05) {
            wa4 = Math.atan2(A, C) * 180 / Math.PI;
        }

        // 1 is FR, 2 is FL, 3 is RL, 4 is RR

        double max = wheelSpeedFrontLeft;
        if (wheelSpeedFrontRight > max)
            max = wheelSpeedFrontRight;
        if (wheelSpeedBackLeft > max)
            max = wheelSpeedBackLeft;
        if (wheelSpeedBackRight > max)
            max = wheelSpeedBackRight;
        if (max > 1) {
            wheelSpeedFrontLeft /= max;
            wheelSpeedFrontRight /= max;
            wheelSpeedBackLeft /= max;
            wheelSpeedBackRight /= max;
        }

        setModules(
            new SwerveModuleState(wheelSpeedFrontLeft, new Rotation2d(wa1)),
            new SwerveModuleState(wheelSpeedFrontRight, new Rotation2d(wa2)),
            new SwerveModuleState(wheelSpeedBackLeft, new Rotation2d(wa3)),
            new SwerveModuleState(wheelSpeedBackRight, new Rotation2d(wa4))
        );
    }

    public void setModules(SwerveModuleState... states){
        moduleFrontLeft .setState(states[0]);
        moduleFrontRight.setState(states[1]);
        moduleBackLeft  .setState(states[2]);
        moduleBackRight .setState(states[3]);
    }
    
    public void setModules(SwerveModuleState state){
        setModules(new SwerveModuleState[]{state, state, state, state});
    }

    public void stop(){
        setModules(new SwerveModuleState());
    }
}
