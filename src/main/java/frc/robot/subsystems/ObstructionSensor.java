package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;

public class ObstructionSensor{
    public DigitalInput sensor;

    public ObstructionSensor(int channel) {
        sensor = new DigitalInput(channel);  
    }

    public boolean isObstructed() {
        return !sensor.get();
    }

    public boolean getRaw(){
        return sensor.get();
    }
}
