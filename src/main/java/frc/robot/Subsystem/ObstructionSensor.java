package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class ObstructionSensor{
    private DigitalInput sensor1;

    public ObstructionSensor(int channel) {
        sensor1 = new DigitalInput(channel);  
    }

    public boolean obstructed() {
        return !sensor1.get();
    }
}
