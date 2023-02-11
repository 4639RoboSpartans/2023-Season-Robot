package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class navXSubsystem extends SubsystemBase{
    private final AHRS ahrs;
    //private final PIDController pid;
    public navXSubsystem(){
        ahrs = new AHRS();
        
        //pid = new PIDController(0, 0, 0);
    }
    public float readPitch(){
        return ahrs.getRoll();
    }

    public double getHeading() {
        return 0;
        //return ahrs.getAngle();
    }

    public boolean isZero(){
        float pitch = ahrs.getRoll();
        if(Math.abs(pitch)<1.0){
            return true;
        }
        return false;
    }
    public void periodic() {
        SmartDashboard.putNumber("AHRS Value", ahrs.getRoll());

    }



    }
