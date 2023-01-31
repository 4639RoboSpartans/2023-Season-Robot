package frc.robot.Subsystem;
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
    
    public void setAllModules(SwerveModuleState state){
        moduleFrontLeft.setDesiredState(state);
        moduleFrontRight.setDesiredState(state);
        moduleBackLeft.setDesiredState(state);
        moduleBackRight.setDesiredState(state);
    }

    @Override
    public void periodic() {
        SwerveModule[] modules = new SwerveModule[]{
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight,
        };

        String[] keys = new String[]{
            "Front Left",
            "Front Right",
            "Back Left",
            "Back Right",
        };
        for(int i = 0; i < modules.length; i++){
            var module = modules[i];
            double degrees = (module.getTrueDegrees() % 360 + 360) % 360;
            String key = keys[i];

            SmartDashboard.putNumber(key, degrees);
        }
    }
}
