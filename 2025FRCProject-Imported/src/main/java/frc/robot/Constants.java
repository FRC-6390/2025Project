// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.auto.JanusConfig;
import frc.robot.utilities.auto.JanusRouteFactory;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModuleConfig;

public interface Constants {
    public interface AUTO{
        
        PIDConfig XY_PID_CONFIG = new PIDConfig(0.7, 0, 0);
        PIDConfig THETA_PID_CONFIG = new PIDConfig(0.02, 0, 0).setContinuous(-Math.PI, Math.PI);
        PIDConfig ROLL_PITCH_PID_CONFIG = new PIDConfig(0.02, 0, 0).setContinuous(-Math.PI, Math.PI);
        PIDConfig ALIGN_XY_PID_CONFIG = new PIDConfig(1, 0, 0);
        PIDConfig ALIGN_THETA_PID_CONFIG = new PIDConfig(0.1, 0, 0).setContinuous(-Math.PI, Math.PI);

        JanusConfig CONFIG = new JanusConfig(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND, SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND, XY_PID_CONFIG, THETA_PID_CONFIG);

        JanusRouteFactory RIGHT_SIDE_SEGMENT_1 = new JanusRouteFactory(CONFIG).to(-1, 0);

        JanusRouteFactory TEST_X_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0).to(-1, 0);
        JanusRouteFactory TEST_Y_AUTO_PATH = new JanusRouteFactory(CONFIG).to(0, 1).to(0, -1).to(0, 0);
        JanusRouteFactory TEST_XY_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0).to(-1, 0).to(0, 0).to(0, 1).to(0, -1).to(0, 0).to(1,1).to(-1,-1).to(1,-1).to(-1,1).to(0,0);
        JanusRouteFactory TEST_THETA_AUTO_PATH = new JanusRouteFactory(CONFIG).to(0, 0,90).to(0, 0, 270).to(0, 0, 180).to(0, 0, 0);
        JanusRouteFactory TEST_MOVEMENT_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 90).to(-1, 0).to(0, 0, 0).to(0, 1, 180).to(0, -1).to(0, 0, 0).to(1,1, 270).to(-1,-1, 90).to(1,-1, 180).to(-1,1, 90).to(0,0, 0);
        JanusRouteFactory TEXT_COMMAND_1_AUTO_PATH = new JanusRouteFactory(CONFIG).run(null);
        JanusRouteFactory TEXT_COMMAND_2_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 0).run(null).to(0, 0, 0);
        JanusRouteFactory TEXT_COMMAND_3_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 0).run(null, true).to(0, 0, 0);

        TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND);
    }
    

    public interface DRIVETRAIN{

        String CANBUS = "can";

        int PIGEON = 20;

        int REV_PDH = 1;

        Translation2d[] SWERVE_MODULE_LOCATIONS = {ROBOT.FRONT_LEFT, ROBOT.FRONT_RIGHT, ROBOT.BACK_LEFT, ROBOT.BACK_RIGHT};

        int FRONT_LEFT_DRIVE = 5;
        int FRONT_RIGHT_DRIVE = 14;
        int REAR_LEFT_DRIVE = 6;
        int REAR_RIGHT_DRIVE = 17;

        int FRONT_LEFT_STEER = 15;
        int FRONT_RIGHT_STEER = 12;
        int REAR_LEFT_STEER = 8;
        int REAR_RIGHT_STEER = 9;

        int FRONT_LEFT_ENCODER = 1;
        int FRONT_RIGHT_ENCODER = 2;
        int REAR_LEFT_ENCODER = 3;
        int REAR_RIGHT_ENCODER = 4;

        

        //Below is what Mathias and I finished with 
        // double FRONT_LEFT_OFFSET = -0.367431640625; 
        // double FRONT_RIGHT_OFFSET =  0.3046875; 
        // double BACK_LEFT_OFFSET = -0.16552734375;
        // double BACK_RIGHT_OFFSET =  -0.379150390625;

        double FRONT_LEFT_OFFSET = 0; 
        double FRONT_RIGHT_OFFSET =  0.0; 
        double BACK_LEFT_OFFSET = -0.0;
        double BACK_RIGHT_OFFSET =  -0.0;



        SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_LEFT_DRIVE, false, FRONT_LEFT_STEER, false, FRONT_LEFT_ENCODER, FRONT_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_RIGHT_DRIVE, false, FRONT_RIGHT_STEER, false, FRONT_RIGHT_ENCODER, FRONT_RIGHT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new SwerveModuleConfig(REAR_LEFT_DRIVE, false, REAR_LEFT_STEER, false, REAR_LEFT_ENCODER, BACK_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(REAR_RIGHT_DRIVE, false, REAR_RIGHT_STEER, false, REAR_RIGHT_ENCODER, BACK_RIGHT_OFFSET, CANBUS);

    }

    public interface SWERVEMODULE {
        double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(17.1);
        double MAX_SPEED_METERS_PER_SECOND_SQUARED = Units.feetToMeters(17.1) * Units.feetToMeters(17.1);
        double MAX_ANGULAR_SPEED_METERS_PER_SECOND = Units.feetToMeters(17.1);
        double MAX_ACCELERATION_METERS_PER_SECOND = 2.75; //15
        double MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND = 3.85;
        double ROTATION_GEAR_RATIO = 1d/(150d/7d);
        double DRIVE_GEAR_RATIO = 1d/(6.12);
        double ROTATION_ENCODER_CONVERSION_RADIANS = ROTATION_GEAR_RATIO * 2 * Math.PI;
        double ROTATION_ENCODER_CONVERSION_RADIANS_PER_SECOND = ROTATION_ENCODER_CONVERSION_RADIANS / 60;
        double DRIVE_ENCODER_CONVERSION_METERS = (DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS); //the 0.625 is a quick fix to correct the odometry
        double DRIVE_ENCODER_CONVERSION_METERS_PER_SECOND = DRIVE_ENCODER_CONVERSION_METERS / 60;
        PIDConfig ROTATION_PID = new PIDConfig(0.5, 0, 0).setContinuous(-Math.PI, Math.PI);
    }

    public interface INTAKE {
        int CENTER_INTAKE_MOTOR = 11;//was 23, changed for testing
        // int BEAM_BREAK = 1;
        int FULL_WIDTH_INTAKE_MOTOR = 19;
        int FEEDNG_ROLLER_MOTOR = 18;

    }

    public interface SHOOTER{
        int RIGHT_SHOOTER_MOTOR = 23; //NOT THE RIGHT VALUES
        int LEFT_SHOOTER_MOTOR = 24; //NOT THE RIGHT VALUES
    }

    

    public interface ARM{
        int ARM_MOTOR_LEFT = 7;
        int ARM_MOTOR_RIGHT = 13;
        double ARM_MAX = -43.7095;
        //0.03
        PIDConfig PID_config = new PIDConfig(0.065, 0.003, 0);
    }
    
    public interface ROBOT {
        double TRACKWIDTH_METERS = 0.61;
        double WHEELBASE_METERS = 0.61;

        int CANDLE_ID = 22; //unkown tbd

        Translation2d FRONT_LEFT = new Translation2d(TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d FRONT_RIGHT = new Translation2d(TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        Translation2d BACK_LEFT = new Translation2d(-TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d BACK_RIGHT = new Translation2d(-TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        int BLINKIN_PORT = 1;
        
        
    }

    public enum APRILTAGS {
    
        INVALID(0, 0),
        
        
        RED_SOURCE_RIGHT(1, -45),
        RED_SOURCE_LEFT(2,-45),
        
        RED_SPEAKER_RIGHT(3, 0),
        RED_SPEAKER_MID(4, 0),

        RED_AMP(5, -90),

        BLUE_AMP(6, 90);

        long id;
        double heading;
        private APRILTAGS(long id, double heading){
            this.id = id;
            this.heading = heading;
            
        }
       
        public static APRILTAGS getByID(long id){
            for (APRILTAGS tag : values()) {
                if(tag.getID() == id) return tag;
            }
            return INVALID;
        }

        public long getID(){
            return id;
        }

        public double getRotation()
        {
            return heading;
        }

    }


  

    }


