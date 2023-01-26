package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase{
    private DigitalInput sensor1;
    public SensorSubsystem() {
        sensor1 = new DigitalInput(1);
        
    }
    public boolean getSensor() {
        return sensor1.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("II", sensor1.get());
    }
    
}
