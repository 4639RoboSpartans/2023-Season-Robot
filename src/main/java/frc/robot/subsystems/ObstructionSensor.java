package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;

public class ObstructionSensor{
    public DigitalInput sensor;
    // public AnalogPotentiometer sensor;

    public ObstructionSensor(int channel) {
        sensor = new DigitalInput(channel); 
        // sensor = new AnalogPotentiometer(channel); 
    }

    public boolean isObstructed() {
        return !sensor.get();
        // if(sensor.get()<0){
        //     return true;
        // }
        // return false;
    }

    public boolean getRaw(){
        return sensor.get();
    }
}
