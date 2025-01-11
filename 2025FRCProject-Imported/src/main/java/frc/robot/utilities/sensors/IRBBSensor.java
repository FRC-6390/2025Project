package frc.robot.utilities.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class IRBBSensor {

    private DigitalInput input;
    
    public IRBBSensor(int port)
    {
        input = new DigitalInput(port);
    }

    public boolean isBroken()
    {
        return !input.get();
    }
    
}
