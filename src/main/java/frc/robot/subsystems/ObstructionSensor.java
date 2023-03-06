package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;

public class ObstructionSensor{
    // public DigitalInput sensor;
    public AnalogPotentiometer distanceSensor;
    public double ObstructionDistance;
    // public AnalogPotentiometer sensor;

    public ObstructionSensor(int channel) {
        // sensor = new DigitalInput(channel); 
        distanceSensor = new AnalogPotentiometer(channel, 10);
        ObstructionDistance = 0; //subject to change
        // sensor = new AnalogPotentiometer(channel); 
    }

    public boolean isObstructed() {
        if(distanceSensor.get()<ObstructionDistance){
            return true;
        }
        return false;
        // return !sensor.get();
    }

    public double getRaw(){
        return distanceSensor.get();
        // return sensor.get();
    }
}
