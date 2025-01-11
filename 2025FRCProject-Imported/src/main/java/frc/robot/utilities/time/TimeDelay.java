package frc.robot.utilities.time;

import edu.wpi.first.wpilibj.Timer;

public class TimeDelay {
    public double startTime;

    public void init(){
        startTime = Timer.getFPGATimestamp();
    
    }

    /**
     * 
     * @param time in seconds
     * @return if time has passed since init
     */
    public boolean hasPassed(double time){
       double current = Timer.getFPGATimestamp();
        if(current-startTime>time){
           return true;
        }
        return false;
    }
}


