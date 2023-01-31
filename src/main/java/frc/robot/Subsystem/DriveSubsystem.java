package frc.robot.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class DriveSubsystem extends SubsystemBase{

    private final SwerveModule
        moduleFrontLeft,
        moduleFrontRight,
        moduleBackLeft,
        moduleBackRight;

    public DriveSubsystem(){
        moduleFrontLeft = new SwerveModule(Constants.IDs.MODULE_FRONT_LEFT);
        moduleFrontRight = new SwerveModule(Constants.IDs.MODULE_FRONT_RIGHT);
        moduleBackLeft = new SwerveModule(Constants.IDs.MODULE_BACK_LEFT);
        moduleBackRight = new SwerveModule(Constants.IDs.MODULE_BACK_RIGHT);
    }
    
    public void setAllModules(SwerveModuleState state){
        moduleFrontLeft.setDesiredState(state);
        moduleFrontRight.setDesiredState(state);
        moduleBackLeft.setDesiredState(state);
        moduleBackRight.setDesiredState(state);
    }
}
