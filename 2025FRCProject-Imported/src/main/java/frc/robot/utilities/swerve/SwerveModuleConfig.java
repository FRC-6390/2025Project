package frc.robot.utilities.swerve;

public record SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor, boolean rotationMotorReversed, int encoder, double encoderOffset, String canbus) {
    
    public SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor, boolean rotationMotorReversed, int encoder, double encoderOffset){
        this(driveMotor, driveMotorReversed, rotationMotor, rotationMotorReversed, encoder, encoderOffset, "can");
    }
}
