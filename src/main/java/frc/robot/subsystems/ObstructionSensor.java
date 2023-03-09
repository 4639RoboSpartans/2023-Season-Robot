package frc.robot.subsystems;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;

public class ObstructionSensor{
    // public DigitalInput sensor;
    // public AnalogPotentiometer distanceSensor;
    public double ObstructionDistance;
    public Debouncer m_Debouncer;
    private final I2C.Port i2cPort;
  private final ColorSensorV3 m_colorSensor;
  double prox;
    // public AnalogPotentiometer sensor;

    public ObstructionSensor(int channel) {
        i2cPort = I2C.Port.kOnboard;
        m_colorSensor = new ColorSensorV3(i2cPort);
        prox = m_colorSensor.getProximity();
        // sensor = new DigitalInput(channel); 
        // distanceSensor = new AnalogPotentiometer(channel, 100000);
        m_Debouncer = new Debouncer(0.05, DebounceType.kBoth);
        ObstructionDistance = 101; //subject to change
        // sensor = new AnalogPotentiometer(channel); 
    }

    public boolean isObstructed() {
        //m_debounce.calculate;
        // m_Debouncer.calculate(
        if(m_Debouncer.calculate(m_colorSensor.getProximity()>ObstructionDistance)){
            return true;
        }
        return false;
        // return !sensor.get();
    }

    public double getRaw(){
        return m_colorSensor.getProximity();
        // return sensor.get();
    }
}
