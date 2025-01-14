package frc.robot.subsystems;

import java.net.ContentHandler;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModule;
import frc.robot.utilities.vision.LimeLight;
import frc.robot.utilities.vision.LimelightHelpers;
import frc.robot.utilities.vision.LimeLight.PoseEstimateType;
import frc.robot.utilities.vision.LimeLight.PoseEstimateWithLatencyType;
 
public class Drivetrain6390 extends SubsystemBase{

  private static SwerveModule[] swerveModules;
  private static boolean hasHeadingBeenSet = false;
  private static double offset = 0;
  private static Boolean isRed = false;
  private LimeLight limeLight;
  private static double absoluteHeading = 0;
  private static PowerDistribution pdh;
  private static Pigeon2 gyro;
  private static ChassisSpeeds chassisSpeeds, feedbackSpeeds;
  private static SwerveDriveKinematics kinematics;
  private static SwerveDriveOdometry odometry;
  private static Pose2d pose;
  private static ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
  private static Field2d gameField;
  private static Field2d gameFieldVision;
  private static Field2d gameFieldVision2;
  private static double desiredHeading;
  private boolean isRobotRelative;
  private boolean driveDisabled;

  public void setRobotRelative(boolean bool) {
    isRobotRelative = bool;
  }
  public void disableDrive(boolean bool) {
    driveDisabled = bool;
  }

  public void XDisable(boolean bool) {
    if(bool)
    {
      disableX = 0;
    }
    else
    {
      disableX = 1;
    }
  }
  public void YDisable(boolean bool) {
    if(bool)
    {
      disableY = 0;
    }
    else
    {
      disableY = 1;
    }
  }
  public void ThetaDisable(boolean bool) {
    if(bool)
    {
      disableTheta = 0;
    }
    else
    {
      disableTheta = 1;
    }
  }

  private static PIDConfig driftCorrectionPID = new PIDConfig(5, 0,0).setContinuous(-Math.PI, Math.PI);
  private static Pose2d visionPose;
  private static PID pid;
  public RobotConfig config; 
  public double disableX = 1;
  public double disableY = 1;
  public double disableTheta = 1;
  //new RobotConfig(
  //   74.088, 
  //   6.883, 
  //   new ModuleConfig(
  //     0.048, 
  //     5.143, 
  //     1.2, 
  //     new DCMotor(12, 
  //                 7, 
  //                 366, 
  //                 2, 
  //                 628.31853, 
  //                 1), 
  //     80, 
  //     1), 
  //     Constants.ROBOT.FRONT_LEFT,
  //     Constants.ROBOT.FRONT_RIGHT,
  //     Constants.ROBOT.BACK_LEFT,
  //     Constants.ROBOT.BACK_RIGHT
  // );
  public SwerveDrivePoseEstimator estimator = 
  new SwerveDrivePoseEstimator(
    kinematics, 
    getRotation2d(), 
    getModulePostions(), 
    new Pose2d(), 
    VecBuilder.fill(0.1,0.1,Units.degreesToRadians(3)), 
    VecBuilder.fill(.475,.475,99999));

  public Drivetrain6390(LimeLight limelight)
  {
    this.limeLight = limelight;
    try{
      config = RobotConfig.fromGUISettings();  }catch(Exception e){
        DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      }
      System.out.println("AutoBuilder Configured");
      AutoBuilder.configure(
        this::getVisionPose, 
        this::resetOdometryVision, 
        this::getSpeeds, 
        this::drive, 
        new PPHolonomicDriveController(
          new PIDConstants(5,0,0),
          new PIDConstants(5,0,0)
        ),
        config,
        () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );}
  
  

  static {
   
  
    gameField = new Field2d();
    gameFieldVision = new Field2d();
    gameFieldVision2 = new Field2d();
 
    SmartDashboard.putData(CommandScheduler.getInstance());
    
   
    swerveModules = new SwerveModule[4];
    swerveModules[0] = new
    SwerveModule(DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG, tab);
    swerveModules[1] = new
    SwerveModule(DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG, tab);
    swerveModules[2] = new
    SwerveModule(DRIVETRAIN.BACK_LEFT_MODULE_CONFIG, tab);
    swerveModules[3] = new
    SwerveModule(DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG, tab);
    gyro = new Pigeon2(DRIVETRAIN.PIGEON, DRIVETRAIN.CANBUS);

    pdh = new PowerDistribution(DRIVETRAIN.REV_PDH, ModuleType.kRev);
    chassisSpeeds = new ChassisSpeeds();
    feedbackSpeeds = new ChassisSpeeds();
    SwerveModulePosition[] SwervePositions =
    {swerveModules[0].getPostion(), swerveModules[1].getPostion(),
    swerveModules[2].getPostion(), swerveModules[3].getPostion()};

    kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);
    odometry = new SwerveDriveOdometry(kinematics,
    Rotation2d.fromDegrees(gyro.getYaw().refresh().getValueAsDouble()), SwervePositions);
    pose = new Pose2d();
    visionPose = new Pose2d();

    pid = new PID(driftCorrectionPID).setMeasurement(() ->
    pose.getRotation().getDegrees());
    gyro.getYaw().refresh().getValueAsDouble();
   absoluteHeading = Math.IEEEremainder(gyro.getYaw().refresh().getValueAsDouble(), 360);
    // tele = new SwerveTelemetry(swerveModules[0], swerveModules[1], swerveModules[2], swerveModules[3], pid, odometry, gameField, tab);
}

  public void init(){
    pdh.clearStickyFaults();
    zeroHeading();
    resetOdometry(new Pose2d(0,0,getRotation2d()));
    
    // shuffleboard();
  }

  public void zeroHeading(){
    if(!hasHeadingBeenSet)
    {
      absoluteHeading = getAbsoluteHeading().getDegrees();
    }
    gyro.setYaw(0);
    if(!hasHeadingBeenSet)
    {
      offset = absoluteHeading -  getHeading();
    }
    resetOdometry(pose);
  }
  public void setOdometryVision(){
    // resetOdometryVision(LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
    resetOdometryVision(limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getPose());
  }
  public double getRate(){
    return gyro.getAngularVelocityZWorld().refresh().getValueAsDouble();
  }

  public Rotation2d getAbsoluteHeading()
  {
    absoluteHeading = getRotation2d().getDegrees();
    return new Rotation2d(absoluteHeading - offset);
  }

  public  void resetHeading()
  {
    if(!hasHeadingBeenSet)
    {
      absoluteHeading = getRotation2d().getDegrees();
    }
    gyro.setYaw(0);
    if(!hasHeadingBeenSet)
    {
      offset = absoluteHeading -  getRotation2d().getDegrees();
    }
  }

  public void setHeading(double heading)
  {
    gyro.setYaw(heading);
  }

  public double getRoll(){
    return Math.IEEEremainder(gyro.getRoll().refresh().getValueAsDouble(), 360);
  }

  public double getPitch(){
    return Math.IEEEremainder(gyro.getPitch().refresh().getValueAsDouble(),360);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw().refresh().getValueAsDouble(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void driftCorrection(ChassisSpeeds speeds){
    if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0) desiredHeading =
pose.getRotation().getDegrees();
    else speeds.omegaRadiansPerSecond += pid.calculate(desiredHeading);
  }



  public Boolean getSide()
  {
    return isRed; 
  }

  public static void updateSide()
  {
    isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
  }

  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }

  public Pose2d getPose(){
    return pose;
  }

  public Pose2d getVisionPose(){
   return visionPose;
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePostions(), pose);
    estimator.resetPosition(getRotation2d(), getModulePostions(), visionPose);
  }
   public void resetOdometryVision(Pose2d pose)
  {
    estimator.resetPosition(getRotation2d(), getModulePostions(), pose);
  }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states,
SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
      //SmartDashboard.putNumber("Module " + i+"",swerveModules[i].getEncoderRadians());
    }
    
  }

  private SwerveModulePosition[] getModulePostions(){
    SwerveModulePosition[] positions = new
SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  public void feedbackDrive(ChassisSpeeds speeds){
    feedbackSpeeds = speeds;
  }

  public void stopWheels(){
    for(int i = 0; i < swerveModules.length; i++){
      swerveModules[i].stop();
     // System.out.println("/////////////////////////////////////|||||||||||||||||||||||||");
    }
  }

  public void lockWheels(){

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].lock();
    }
  }

  public void unlockWheels(){
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].unlock();
    }
  }

 private void updateOdometry(){
      odometry.update(getRotation2d(), getModulePostions());
      pose = odometry.getPoseMeters();

      boolean doRejectUpdate = false;
      estimator.update(getRotation2d(), getModulePostions());
      
      Pose2d roboPos = limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getPose();
      int tagCount = (int)limeLight.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getTagCount();
      if(Math.abs(gyro.getRate()) > 720) 
      {
        doRejectUpdate = true;
      }
      if(tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        estimator.addVisionMeasurement(roboPos,  edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
      }
    
    gameField.setRobotPose(pose);
    gameFieldVision2.setRobotPose(roboPos);
    visionPose = estimator.getEstimatedPosition();
    gameFieldVision.setRobotPose(visionPose);

  }

  public ChassisSpeeds getSpeeds()
  {
    return chassisSpeeds;
  }


  public double maxAccel = 0;
  
  @Override
  public void periodic() {
    if(driveDisabled)
    {
      chassisSpeeds = new ChassisSpeeds(0,0,0);
    }
    double xSpeed = chassisSpeeds.vxMetersPerSecond * disableX +
feedbackSpeeds.vxMetersPerSecond;
    double ySpeed = chassisSpeeds.vyMetersPerSecond * disableY +
feedbackSpeeds.vyMetersPerSecond;
    double thetaSpeed = chassisSpeeds.omegaRadiansPerSecond * disableTheta +
feedbackSpeeds.omegaRadiansPerSecond;
    ChassisSpeeds speed = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    
    SwerveModuleState[] states;
    // if(!isRobotRelative)
    // {
    states = kinematics.toSwerveModuleStates(speed);
    // }
    // else
    // {
    // states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(speed, getRotation2d()));
    // }
    setModuleStates(states);

    updateOdometry();
    driftCorrection(speed);
    
    if(gyro.getAccelerationX().getValueAsDouble() > maxAccel)
    {
      maxAccel = gyro.getAccelerationX().getValueAsDouble();
    }
    
  }

  @Override
  public void simulationPeriodic() {

  }
  
}