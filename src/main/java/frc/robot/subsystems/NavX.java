package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class NavX extends SubsystemBase{
    private final AHRS ahrs;
    public NavX(){
        ahrs = new AHRS();
    }
    public float readPitch(){
        return ahrs.getRoll();
    }

    public double getHeading() {
        return 0;
    }

    public boolean isZero(){
        double pitch = ahrs.getRoll();
        return Math.abs(pitch) < 1.0;
    }
    public void periodic() {
        SmartDashboard.putNumber("AHRS Value", ahrs.getRoll());
    }
}
