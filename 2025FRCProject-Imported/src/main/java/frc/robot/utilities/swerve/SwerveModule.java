package frc.robot.utilities.swerve;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;


public class SwerveModule {
    public TalonFX driveMotor;
    public TalonFX rotationMotor;

    private CANcoder encoder;
    private PID pid;
    
    private GenericEntry offsetEntry;

    private double encoderOffset;
   // private REVMaglimitSwitch limitSwitch;
    private static int instances = 0;

   private StatusSignal<Angle> drivePos, encoderPos;
   private StatusSignal<AngularVelocity> driveVel;
    public SwerveModule(SwerveModuleConfig config){
        this(config, null);
    }

    public static PIDController rotationPidController = new PIDController(0.25, 0, 0);
   
    

    public SwerveModule(SwerveModuleConfig config, ShuffleboardTab tab){
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        if(config.canbus() != null){
           // System.out.println(config.canbus());
            driveMotor = new TalonFX(config.driveMotor(), config.canbus());
            rotationMotor = new TalonFX(config.rotationMotor(), config.canbus());
            encoder = new CANcoder(config.encoder(), config.canbus());
        }else{
            driveMotor = new TalonFX(config.driveMotor());
            rotationMotor = new TalonFX(config.rotationMotor());
            encoder = new CANcoder(config.encoder());
        }
        encoderOffset = config.encoderOffset();
        TalonFXConfiguration con2 =  new TalonFXConfiguration();
        CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
        curr.SupplyCurrentLimitEnable = true;
        curr.SupplyCurrentLimit = 50; //used to be 60
        con2.withCurrentLimits(curr);

        driveMotor.getConfigurator().apply(con2);
        encoderPos=encoder.getPosition();
        
        if(offsetEntry != null) encoderOffset = offsetEntry.getDouble(0.0);
        
        CANcoderConfiguration  con = new CANcoderConfiguration();
        con.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        con.MagnetSensor.MagnetOffset = encoderOffset;
        encoder.getConfigurator().apply(con);
        pid = new PID(SWERVEMODULE.ROTATION_PID).setMeasurement(() -> getRotationMotorPosition());
        //if(tab != null){
            // ShuffleboardLayout layout = tab.getLayout("Swerve Module "+instances, BuiltInLayouts.kList).withSize(2, 6);
            // layout.add(pid);
            // offsetEntry = layout.add("Offset "+ instances, getEncoderOffset()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -Math.PI, "max", Math.PI)).getEntry();
            // layout.addDouble("Angle "+instances, () -> getPostion().angle.getDegrees());
            // layout.addDouble("Absolute "+instances, () -> getAbsolutePosition());
       // }
        instances++;
        drivePos=driveMotor.getRotorPosition();
        driveVel=driveMotor.getRotorVelocity();
        resetEncoders();
        lock();
    }

    public double getDriveMotorVelocity(){

        return driveVel.getValueAsDouble()  * SWERVEMODULE.DRIVE_ENCODER_CONVERSION_METERS;
        
    }
    
    public double getDriveMotorPosition(){

       return drivePos.getValueAsDouble()* SWERVEMODULE.DRIVE_ENCODER_CONVERSION_METERS;
    }

    public double getRotationMotorPosition(){
        return getEncoderRadians();
    }

    public double getAbsolutePosition(){
        return encoderPos.getValueAsDouble() * Math.PI/180d;
    }

    public double getEncoderOffset(){
        return encoderOffset;
    }

    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

    public double getEncoderRadians(){
        return (encoderPos.getValueAsDouble()*360 * Math.PI/180d);
    }

    public void resetEncoders(){
        driveMotor.setPosition(0);
        rotationMotor.setPosition(encoderPos.getValueAsDouble());
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getEncoderRadians()));
    }

    public SwerveModulePosition getPostion(){
        return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getEncoderRadians()));
    }

    public void setDriveMotor(double speed){
        driveMotor.set(speed);
    }

    public void setRotationMotor(double speed){
        rotationMotor.set(speed);
    }

    public void setDesiredState(SwerveModuleState state){
        refresh();
        
    

        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();

            return;
        }

      state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);

        rotationMotor.set(rotationPidController.calculate(-getEncoderRadians(), -state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void setToAngle(double angle){
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(angle));

        rotationMotor.set(rotationPidController.calculate(-getEncoderRadians(), -state.angle.getRadians()));
    }

    public void lock(){
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void unlock(){
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        rotationMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void refresh(){
        drivePos.refresh();
        driveVel.refresh();
        encoderPos.refresh();
    }

}