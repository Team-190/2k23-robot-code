// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    //general drive constants
    public static final int PIGEON_CHANNEL = 1;
    public static final int LEFT_LEADER_CHANNEL = 20;
    public static final int LEFT_FOLLOWER_CHANNEL = 21;
    public static final int RIGHT_LEADER_CHANNEL = 10;
    public static final int RIGHT_FOLLOWER_CHANNEL = 11;

    public static final double TRACKWIDTH_METERS = 0.6382; // horizontal distance between wheels
    public static final double TICKS_PER_MOTOR_REVOLUTION = 2048;
    //TODO: change to 6 in wheels
    public static final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inch diameter in meters
    public static final double AUTO_P = 0; // Calculated by SysID

    // (14/58) ratio to (20/28) on the drivetrain gearbox
    public static final double WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS = 0.161716; // (0.172)
    public static final double METERS_PER_TICK =
        (1 / TICKS_PER_MOTOR_REVOLUTION)
        * // MOTOR ROTATIONS per count
        WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS
        * (WHEEL_DIAMETER_METERS * Math.PI);
    
    //auto drive constants
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TRACKWIDTH_METERS);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kPDriveVel = 0;

  }
}
