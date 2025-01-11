package frc.robot.utilities.auto;

import frc.robot.utilities.controlloop.PIDConfig;

public record JanusConfig(double maxSpeedMeters, double maxAccelerationMeters, double maxAngularSpeedMeters, double maxAngularAccelerationMeters, PIDConfig xyPIDConfig, PIDConfig thetaPIDConfig) {
    

}
