package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    public void setModules(SwerveModuleState... states){
        moduleFrontLeft.setDesiredState(states[0]);
        moduleFrontRight.setDesiredState(states[1]);
        moduleBackLeft.setDesiredState(states[2]);
        moduleBackRight.setDesiredState(states[3]);
    }
    
    public void setModules(SwerveModuleState state){
        setModules(new SwerveModuleState[]{state, state, state, state});
    }

    public void stop(){
        setModules(new SwerveModuleState());
    }
}
