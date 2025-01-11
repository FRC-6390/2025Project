package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.sensors.IRBBSensor;
 
public class Intake extends SubsystemBase {
 
  public static TalonFX centerIntakeRoller;
  public static TalonFX fullWidthIntakeRoller;
  public static TalonFX feedingRollers;
  public static double feedingRollersSpeed, fullWidthIntakeRollerSpeed, centerIntakeRollerSpeed;

  // public static IRBBSensor lowerIntakeBeamBreak;
  public static IRBBSensor upperIntakeBeamBreak;
  public static boolean rollersEnabled, override, reversed;
 
 
  public static boolean isReversed() {
    return reversed;
  }

  public static void setReversed(boolean reversed) {
    Intake.reversed = reversed;
  }

  public boolean isOverride() {
    return override;
  }

  public void setOverride(boolean override) {
    this.override = override;
  }

  public boolean isRollersEnabled() {
    return rollersEnabled;
  }

  public void setRollersEnabled(boolean rollersEnabled) {
    this.rollersEnabled = rollersEnabled;
  }

  public Intake()
  {

  }
 
  static
  {
    
  centerIntakeRoller = new TalonFX(Constants.INTAKE.CENTER_INTAKE_MOTOR, Constants.DRIVETRAIN.CANBUS);
  fullWidthIntakeRoller = new TalonFX(Constants.INTAKE.FULL_WIDTH_INTAKE_MOTOR, Constants.DRIVETRAIN.CANBUS);
  // lowerIntakeBeamBreak = new IRBBSensor(Constants.INTAKE.BEAM_BREAK);
  upperIntakeBeamBreak = new IRBBSensor(4);
  feedingRollers = new TalonFX(Constants.INTAKE.FEEDNG_ROLLER_MOTOR, Constants.DRIVETRAIN.CANBUS);
 
  TalonFXConfiguration con2 =  new TalonFXConfiguration();
  CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
  curr.SupplyCurrentLimitEnable = true;
  curr.SupplyCurrentLimit = 40;
  con2.withCurrentLimits(curr);
  override = false;

  feedingRollers.getConfigurator().apply(con2);
  fullWidthIntakeRoller.getConfigurator().apply(con2);
  centerIntakeRoller.getConfigurator().apply(con2);
}
 
  // //Get value of the intake lift limit switch
  // public static boolean getLowerBeamBreak()
  // {
  //   //false for triggered, otherwise true
  // return lowerIntakeBeamBreak.isBroken();
  // }
 
   public boolean hasNote(){
 
    return upperIntakeBeamBreak.isBroken();
   }

  public void setHome()
  {
    centerIntakeRoller.setPosition(0);
  }
  public double getRotations()
  {
    return Math.abs(centerIntakeRoller.getRotorPosition().refresh().getValueAsDouble());
  }
  public void feed(double speed)
  {
    // feedingRollers.set(speed);
    feedingRollersSpeed = speed;
  }
  public void centerIntake(double speed)
  {
    // centerIntakeRoller.set(speed);
    centerIntakeRollerSpeed = speed;
  }
 
  public void fullWidth(double speed)
  {
    // fullWidthIntakeRoller.set(0);
    fullWidthIntakeRollerSpeed = speed;
  }
  //Sets the intake rollers
  // public void setRollers(double speed)
  // {
  //   if(num == 1)
  //   {
  //   centerIntakeRoller.set(0);
  //   fullWidthIntakeRoller.set(0);
  //   feedingRollers.set(0);
  //  //Beam is BROKEN
  //   }
  //   else if(num == 2)
  //   {
  //   centerIntakeRoller.set(speed - 0.2);
  //   fullWidthIntakeRoller.set(speed);
  //   feedingRollers.set(speed + 0.4);
  //  // System.out.println("_____________________________-Beam is INTACT -___________________________________");
  //   }
 
  // }
 
  public static StatusSignal<Current> getRollerCurrent()
  {
    return centerIntakeRoller.getSupplyCurrent();
  }

  public void stopAll()
  {
    fullWidthIntakeRollerSpeed = 0;
    centerIntakeRollerSpeed = 0;
    feedingRollersSpeed = 0;
  }
  public void update()
  {
    if(hasNote())
    {
      SmartDashboard.putNumber("Lights", 1);
    }
    else
    {
      SmartDashboard.putNumber("Lights", 0);
    }
    if(isRollersEnabled() || isOverride())
    {
  
      if(reversed)
      {
        feedingRollers.set(-feedingRollersSpeed);
        centerIntakeRoller.set(-centerIntakeRollerSpeed);
        fullWidthIntakeRoller.set(-fullWidthIntakeRollerSpeed);
      } 
      else
      {
        feedingRollers.set(feedingRollersSpeed);
        centerIntakeRoller.set(centerIntakeRollerSpeed);
        fullWidthIntakeRoller.set(fullWidthIntakeRollerSpeed);
        // System.out.println("INTAKING");
      }
    }
    else
    {
      feedingRollers.set(0);
      centerIntakeRoller.set(0);
      fullWidthIntakeRoller.set(0); 
    }
  }
 

  @Override
  public void periodic()
  {
   update();
  //  SmartDashboard.putBoolean("Game Piece", hasNote());

   }
}