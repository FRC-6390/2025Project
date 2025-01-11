package frc.robot.utilities.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DebouncedButton extends Trigger {
    private static float DEFUALT_DEBOUNCE_PERIOD = 0.5f;
    private Debouncer debouncer;

    public DebouncedButton(GenericHID joystick, int buttonNumber) {
        this(joystick, buttonNumber,DEFUALT_DEBOUNCE_PERIOD);
    }

    public DebouncedButton(GenericHID joystick, int buttonNumber, float debouncePeriod) {
        super(() -> joystick.getRawButton(buttonNumber));
        debouncer = new Debouncer(this, debouncePeriod);
    }

    public boolean debounced() {
        return debouncer.get();
    }
    
}
