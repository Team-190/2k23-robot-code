package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.TalonPIDConfig;

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
        public static final int LEFT_JOYSTICK_CHANNEL = 0;
        public static final int RIGHT_JOYSTICK_CHANNEL = 1;
        public static final int XBOX_CHANNEL = 2;
    }
    public static final class SensorConstants {
        public static final int GYRO_CHANNEL = 1;
        public static final int PH_CHANNEL = 2;
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

        // Encoder and PID Constants (For Auto)
        public static final double TRACKWIDTH_METERS = 0.6382; // horizontal distance between wheels
        public static final double COUNTS_PER_MOTOR_REVOLUTION = 2048;
        public static final double WHEEL_DIAMETER_METERS = 0.1021; // 4 inch diameter in meters
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

    public static final class ClawConstants {
        public static final int CLAW_MOTOR_CHANNEL = 7; // update when motor is finalized
        public static final int CLAW_SOLENOID_CHANNEL_1 = 0;
        public static final int CLAW_SOLENOID_CHANNEL_2 = 1;
        public static final int CLAW_LIFT_SOLENOID_CHANNEL = 2;
        public static final double CLAW_SPEED = 1;
    }

    public static final class ArmConstants {
        public static final int LIMIT_SWITCH_CHANNEL = 9;

        public static final int ARM_MOTOR_CHANNEL = 6; // update when motor is finalized
        public static final double ARM_SPEED = 0.5;
        public static final double ARM_SCORING_POSITION = 57; // in inches -- should be checked
        public static final double ARM_COLLECTING_POSITION = 50; // in inches -- should be checked
       
        public static final double ARM_MAX_RPM = 100; // update
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;
        public static final int ARM_TICKS_PER_ROTATION = 4096;
        public static final int SLOT_ID = 0;
        public static final double ARM_MOTOR_VELOCITY = 5250; // update
        public static final double ARM_MOTOR_ACCELERATION = ARM_MOTOR_VELOCITY*2; // update
        public static final int ARM_MOTOR_MOTION_SMOOTHING = 3; // update
        public static final double TOLERANCE = 500;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final int TICKS_PER_INCH = 100; // update
        public static final int MAX_EXTENSION_TICKS = 1000; // update?
        public static final int MIN_EXTENSION_TICKS = 0;
        public static final boolean INVERT_MOTOR = false;
        public static final boolean SENSOR_PHASE = false;
        public static final boolean ENABLE_SOFT_LIMITS = true;

        public TalonPIDConfig ah;
        
        public static TalonPIDConfig ARM_PID_CONFIG = new TalonPIDConfig(SENSOR_PHASE, INVERT_MOTOR, ARM_TICKS_PER_ROTATION,
                        P, I, D, F, TOLERANCE, 
                        MIN_EXTENSION_TICKS, MAX_EXTENSION_TICKS, ENABLE_SOFT_LIMITS,
                        ARM_MOTOR_VELOCITY, ARM_MOTOR_ACCELERATION, ARM_MOTOR_MOTION_SMOOTHING);
               

         
      }

    public static final class PivotConstants {
        public static final int LIMIT_SWITCH_CHANNEL = 11;
        
        public static final int PIVOT_MOTOR_CHANNEL = 7; // update when motor is finalized
        public static final double PIVOT_SPEED = 0.5;
        public static final double PIVOT_SCORING_POSITION = 24; // in inches -- should be checked
        public static final double PIVOT_COLLECTING_POSITION = 10; // in inches -- should be checked
        
        public static final double PIVOT_MAX_RPM = 6380; // update
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;
        public static final int PIVOT_TICKS_PER_ROTATION = 2048;
        public static final int SLOT_ID = 0;
        public static final double PIVOT_MOTOR_VELOCITY = 5250; // update
        public static final double PIVOT_MOTOR_ACCELERATION = PIVOT_MOTOR_VELOCITY*2; // update
        public static final int PIVOT_MOTOR_MOTION_SMOOTHING = 0; // update
        public static final double TOLERANCE = 10;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final int TICKS_PER_INCH = 100; // update
        public static final int MAX_ANGLE_TICKS = 90; // update?
        public static final int MIN_ANGLE_TICKS = -90;
        public static final boolean INVERT_MOTOR = false;
        public static final boolean SENSOR_PHASE = false;
        public static final boolean ENABLE_SOFT_LIMITS = true;

        public static final TalonPIDConfig PIVOT_PID_CONFIG = new TalonPIDConfig(SENSOR_PHASE, INVERT_MOTOR, PIVOT_TICKS_PER_ROTATION,
                        P, I, D, F, TOLERANCE, 
                        MIN_ANGLE_TICKS, MAX_ANGLE_TICKS, ENABLE_SOFT_LIMITS,
                        PIVOT_MOTOR_VELOCITY, PIVOT_MOTOR_ACCELERATION, PIVOT_MOTOR_MOTION_SMOOTHING);
               

    }

    public static final class WristConstants {
        public static final int WRIST_MOTOR_CHANNEL = 8; // update when motor is finalized
        public static final double WRIST_SPEED = 0.5;
        public static final double WRIST_SCORING_POSITION = 4; // in inches -- should be checked
        public static final double WRIST_COLLECTING_POSITION = 57; // in inches -- should be checked
    
    public static final double WRIST_MAX_RPM = 6380; // update
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;
        public static final int WRIST_TICKS_PER_ROTATION = 2048;
        public static final int SLOT_ID = 0;
        public static final double WRIST_MOTOR_VELOCITY = 5250; // update
        public static final double WRIST_MOTOR_ACCELERATION = WRIST_MOTOR_VELOCITY*2; // update
        public static final int WRIST_MOTOR_MOTION_SMOOTHING = 0; // update
        public static final double TOLERANCE = 10;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final int TICKS_PER_INCH = 100; // update
        public static final int MAX_ANGLE_TICKS = 90; // update?
        public static final int MIN_ANGLE_TICKS = -90;
        public static final boolean INVERT_MOTOR = false;
        public static final boolean SENSOR_PHASE = false;
        public static final boolean ENABLE_SOFT_LIMITS = true;

        public static final TalonPIDConfig WRIST_PID_CONFIG = new TalonPIDConfig(SENSOR_PHASE, INVERT_MOTOR, WRIST_TICKS_PER_ROTATION,
                        P, I, D, F, TOLERANCE, 
                        MIN_ANGLE_TICKS, MAX_ANGLE_TICKS, ENABLE_SOFT_LIMITS,
                        WRIST_MOTOR_VELOCITY, WRIST_MOTOR_ACCELERATION, WRIST_MOTOR_MOTION_SMOOTHING);
               

    }

    public static final class LEDConstants {
        public static final int LED_PORT = 5; // change
        public static final int LED_LENGTH = 60;
        public static final int LED_COUNT = 300;
        public static final int CANdleID = 10;
    }

    

}
