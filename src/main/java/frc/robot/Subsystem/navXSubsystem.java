package frc.robot.Subsystem;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class navXSubsystem extends SubsystemBase{
    private final AHRS ahrs;
    private final PIDController pid;
    public navXSubsystem(){
        ahrs = new AHRS();
        pid = new PIDController(0, 0, 0);
    }
    public float readPitch(){
        return ahrs.getPitch();
    }

    public boolean isZero(){
        float pitch = ahrs.getPitch();
        if(Math.abs(pitch)<1.0){
            return true;
        }
        return false;
    }



    }
