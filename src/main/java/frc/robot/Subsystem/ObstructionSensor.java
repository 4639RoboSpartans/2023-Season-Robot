package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class ObstructionSensor{
    private DigitalInput sensor;

    public ObstructionSensor(int channel) {
        sensor = new DigitalInput(channel);  
    }

    public boolean isObstructed() {
        return !sensor.get();
    }
}
