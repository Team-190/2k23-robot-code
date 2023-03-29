package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.utils.TalonPIDConfig;

public final class Constants {

    /** Constants for the Inputs, like Attack 3s and Xbox Controllers */
    public static final class InputConstants {

        public enum INPUT_METHOD {
            CONTROLLER,
            STICKS,
            KINECT,
            BUTTON_BOX
        }

        // USB Ids
        public static final int LEFT_JOYSTICK_CHANNEL = 2;
        public static final int RIGHT_JOYSTICK_CHANNEL = 3;
        public static final int XBOX_DR_CHANNEL = 0;
        public static final int XBOX_OP_CHANNEL = 1;
    }
    public static final class SensorConstants {
        public static final int GYRO_CHANNEL = 7;
    }



    /**
     * Constants for the drivetrainSubsystem
     */
    public static final class DrivetrainConstants {

        public enum DRIVE_STYLE {
            ARCADE,
            TANK,
            MCFLY // Curvature
        }
        public enum DRIVE_INPUT {
            JOYSTICKS,
            CONTROLLER
        }

        // CAN Ids
        public static final int LEFT_LEADER_CHANNEL = 20;
        public static final int LEFT_FOLLOWER_CHANNEL = 21;
        public static final int RIGHT_LEADER_CHANNEL = 10;
        public static final int RIGHT_FOLLOWER_CHANNEL = 11;

        // PID Constants (Not Auto Constants)
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;

        //Acceleration Smoothing
        public static final double ACCEL_LIMIT = 0.35;

        // Encoder and PID Constants (For Auto)
        public static final double TRACKWIDTH_METERS = 0.6382; // horizontal distance between wheels
        public static final double COUNTS_PER_MOTOR_REVOLUTION = 2048;
        public static final double WHEEL_DIAMETER_METERS = 0.1021; // 4 inch diameter in meters // TODO: Change this
        public static final double AUTO_P = 2.1989; // Calculated by SysID

        // (14/58) ratio to (20/28) on the drivetrain gearbox
        public static final double WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS = 0.161716; // (0.172)
        public static final double METERS_PER_COUNT =
                (1 / COUNTS_PER_MOTOR_REVOLUTION)
                        * // MOTOR ROTATIONS per count
                        WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS
                        * (WHEEL_DIAMETER_METERS * Math.PI);

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(TRACKWIDTH_METERS);
        public static final double RAMSETE_B = 2; 
        public static final double RAMSETE_ZETA = 0.7;

        // Max Speed Constants
        public static final double MAX_SPEED_METERS_PER_SECOND = 2.7432; // Max speed set as 9 ft/s
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
        public final static int MAX_VOLTAGE = 11;

        // Constants calculated by System Identification software
        public static final double S_VOLTS = 0.65089; 
        public static final double V_VOLT_SECONDS_PER_METER = 1.938;
        public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.14035;

        public static final SimpleMotorFeedforward DRIVE_FEED_FORWARD =
                new SimpleMotorFeedforward(
                        DrivetrainConstants.S_VOLTS,
                        DrivetrainConstants.V_VOLT_SECONDS_PER_METER,
                        DrivetrainConstants.A_VOLT_SECONDS_SQUARED_PER_METER);

                        
        // Create a voltage constraint to ensure we don't accelerate too fast
        public final static DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
            DrivetrainConstants.DRIVE_FEED_FORWARD, DrivetrainConstants.DRIVE_KINEMATICS, DrivetrainConstants.MAX_VOLTAGE);
                
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_CHANNEL = 6; // update when motor is finalized
        public static final double INTAKE_SPEED = 1;
    }

    public static final class ArmConstants {
        public static final int LIMIT_SWITCH_CHANNEL = 0;

        public static final int ARM_MOTOR_CHANNEL = 4; // update when motor is finalized
        public static final double ARM_SPEED = 0.5;
        //public static final double ARM_SCORING_POSITION = 57; // in inches -- should be checked
        public static final double ARM_COLLECTING_POSITION = 50; // in inches -- should be checked
       
        public static final double ARM_MAX_RPM = 6380; // update
        public static final int ARM_TICKS_PER_ROTATION = 2048;
        public static final double P = 0.055;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 1023 / ((ARM_MAX_RPM * ARM_TICKS_PER_ROTATION) / 600);
        public static final int SLOT_ID = 0;
        public static final double ARM_MOTOR_VELOCITY = ARM_MAX_RPM/1.5; 
        public static final double ARM_MOTOR_ACCELERATION = ARM_MAX_RPM*2.5; 
        public static final int ARM_MOTOR_MOTION_SMOOTHING = 4; // update
        public static final double TOLERANCE = 1000;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final int TICKS_PER_INCH = 100; // update
        public static final int MAX_EXTENSION_TICKS = 135000; 
        public static final int MIN_EXTENSION_TICKS = 0;
        public static final boolean INVERT_MOTOR = false;
        public static final boolean SENSOR_PHASE = false;
        public static final boolean ENABLE_SOFT_LIMITS = true;
        public static final int MIDDLE_POSITION = 85000; 

        // extension
        public static final int LOW_EXT_TICKS = 0; // likely the same for scoring, pickup, and cone/cube
        public static final int SINGLE_PICKUP_EXT_TICKS = 0;

        public static final int CONE_DOUBLE_PICKUP_EXT_TICKS = 90;
        public static final int CONE_MID_GOAL_EXT_TICKS = 53000; // was 90000
        public static final int CONE_HIGH_GOAL_EXT_TICKS = 134900;

        public static final int CUBE_DOUBLE_PICKUP_EXT_TICKS = 90;
        public static final int CUBE_MID_GOAL_EXT_TICKS = 0;
        public static final int CUBE_HIGH_GOAL_EXT_TICKS = 90000;

        
        public static TalonPIDConfig ARM_PID_CONFIG = new TalonPIDConfig(SENSOR_PHASE, INVERT_MOTOR, ARM_TICKS_PER_ROTATION,
                        P, I, D, F, TOLERANCE, 
                        MIN_EXTENSION_TICKS, MAX_EXTENSION_TICKS, ENABLE_SOFT_LIMITS,
                        ARM_MOTOR_VELOCITY, ARM_MOTOR_ACCELERATION, ARM_MOTOR_MOTION_SMOOTHING);
               

         
      }

    public static final class PivotConstants {
        public static final int LIMIT_SWITCH_CHANNEL = 11;
        
        public static final int PIVOT_MOTOR_CHANNEL = 3; // update when motor is finalized
        public static final double PIVOT_SPEED = 0.5;
        public static final double PIVOT_SCORING_POSITION = 24; // in inches -- should be checked
        public static final double PIVOT_COLLECTING_POSITION = 10; // in inches -- should be checked
        
        public static final double PIVOT_MAX_RPM = 6380; // update
        public static final int PIVOT_TICKS_PER_ROTATION = 2048;
        public static final double P = 0.02;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 1023 / ((PIVOT_MAX_RPM * PIVOT_TICKS_PER_ROTATION) / 600);
        //1023 / rpmToTicksPer100ms(m_maxRPM);
        public static final int SLOT_ID = 0;
        public static final double PIVOT_MOTOR_VELOCITY = PIVOT_MAX_RPM/1.25; // update
        public static final double PIVOT_MOTOR_ACCELERATION = PIVOT_MAX_RPM*2; // update
        public static final int PIVOT_MOTOR_MOTION_SMOOTHING = 0; // update
        public static final double TOLERANCE = 2000; // ticks
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final int TICKS_PER_DEGREE = 100; // update
        public static final int MAX_ANGLE_TICKS = 240000; // update?
        public static final int MIN_ANGLE_TICKS = -240000;
        public static final boolean INVERT_MOTOR = false;
        public static final boolean SENSOR_PHASE = false;
        public static final boolean ENABLE_SOFT_LIMITS = true;

        //pivot
        public static final int LOW_PIVOT_TICKS = -240000; // likely the same for scoring, pickup, and cone/cube
        public static final int SINGLE_PICKUP_PIVOT_TICKS = -172500; //can and should be the same for cone/cube

        public static final int CONE_DOUBLE_PICKUP_PIVOT_TICKS = 90;
        public static final int CONE_MID_GOAL_PIVOT_TICKS = -85000; // was 80000
        public static final int CONE_HIGH_GOAL_PIVOT_TICKS = -80000;

        public static final int CUBE_DOUBLE_PICKUP_PIVOT_TICKS = 90;
        public static final int CUBE_MID_GOAL_PIVOT_TICKS = -126500;
        public static final int CUBE_HIGH_GOAL_PIVOT_TICKS = -120000;


  
  

        

        public static final TalonPIDConfig PIVOT_PID_CONFIG = new TalonPIDConfig(SENSOR_PHASE, INVERT_MOTOR, PIVOT_TICKS_PER_ROTATION,
                        P, I, D, F, TOLERANCE, 
                        MIN_ANGLE_TICKS, MAX_ANGLE_TICKS, ENABLE_SOFT_LIMITS,
                        PIVOT_MOTOR_VELOCITY, PIVOT_MOTOR_ACCELERATION, PIVOT_MOTOR_MOTION_SMOOTHING);
    

    }

    public static final class WristConstants {
        public static final int WRIST_MOTOR_CHANNEL = 5; // update when motor is finalized
        public static final double WRIST_SPEED = 0.5;
        public static final double WRIST_SCORING_POSITION = 4; // in inches -- should be checked
        public static final double WRIST_COLLECTING_POSITION = 57; // in inches -- should be checked
    
    public static final double WRIST_MAX_RPM = 6380; // update
    public static final int WRIST_TICKS_PER_ROTATION = 2048;
        public static final double P = 0.02;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 1023 / ((WRIST_MAX_RPM * WRIST_TICKS_PER_ROTATION) / 600);
        public static final int SLOT_ID = 0;
        public static final double WRIST_MOTOR_VELOCITY = WRIST_MAX_RPM/1.25; // update
        public static final double WRIST_MOTOR_ACCELERATION = WRIST_MAX_RPM; // update
        public static final int WRIST_MOTOR_MOTION_SMOOTHING = 2; // update
        public static final double TOLERANCE = 1000;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final int TICKS_PER_INCH = 100; // update
        public static final int MAX_ANGLE_TICKS = 150000; // update?
        public static final int MIN_ANGLE_TICKS = -145000;
        public static final boolean INVERT_MOTOR = false;
        public static final boolean SENSOR_PHASE = false;
        public static final boolean ENABLE_SOFT_LIMITS = true;
        
       //wrist
        public static final int FORWARD_RIGHT_ANGLE = -140000;
        public static final int BACKWARD_RIGHT_ANGLE = 145000;

        public static final int LOW_WRIST_TICKS = 0; // likely the same for scoring, pickup, and cone/cube
        public static final int SINGLE_PICKUP_WRIST_TICKS = 0; //can and should be the same for cone, cube

        public static final int CONE_DOUBLE_PICKUP_WRIST_TICKS = 90;
        public static final int CONE_MID_GOAL_WRIST_TICKS = -90000;
        public static final int CONE_HIGH_GOAL_WRIST_TICKS = 90;

        public static final int CUBE_DOUBLE_PICKUP_WRIST_TICKS = 90;
        public static final int CUBE_MID_GOAL_WRIST_TICKS = -30000;
        public static final int CUBE_HIGH_GOAL_WRIST_TICKS = 0;

  

        


        public static final TalonPIDConfig WRIST_PID_CONFIG = new TalonPIDConfig(SENSOR_PHASE, INVERT_MOTOR, WRIST_TICKS_PER_ROTATION,
                        P, I, D, F, TOLERANCE, 
                        MIN_ANGLE_TICKS, MAX_ANGLE_TICKS, ENABLE_SOFT_LIMITS,
                        WRIST_MOTOR_VELOCITY, WRIST_MOTOR_ACCELERATION, WRIST_MOTOR_MOTION_SMOOTHING);
               

    }

    public static final class LEDConstants {
        public static final int LED_PORT = 5; // change
        public static final int LED_LENGTH = 60;
        public static final int LED_COUNT = 300;
        public static final int CANdleID = 8;
    }

    

}
