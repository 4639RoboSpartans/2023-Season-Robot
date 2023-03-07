package frc.robot.subsystems;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;

public class ObstructionSensor{
    // public DigitalInput sensor;
    public AnalogPotentiometer distanceSensor;
    public double ObstructionDistance;
    public Debouncer m_Debouncer;
    // public AnalogPotentiometer sensor;

    public ObstructionSensor(int channel) {
        // sensor = new DigitalInput(channel); 
        distanceSensor = new AnalogPotentiometer(channel, 10);
        m_Debouncer = new Debouncer(0.1, DebounceType.kBoth);
        ObstructionDistance = 0; //subject to change
        // sensor = new AnalogPotentiometer(channel); 
    }

    public boolean isObstructed() {
        //m_debounce.calculate;
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
