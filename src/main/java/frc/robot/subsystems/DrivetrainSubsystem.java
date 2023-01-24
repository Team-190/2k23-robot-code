// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  // The motors on the left side of the drive.
  public final WPI_TalonFX leftLeader = new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER_CHANNEL);
  private final WPI_TalonFX leftFollower =
          new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER_CHANNEL);

  public final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER_CHANNEL);
  private final WPI_TalonFX rightFollower =
          new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER_CHANNEL);

  private final MotorControllerGroup leftSide = new MotorControllerGroup(leftLeader, leftFollower);
  private final MotorControllerGroup rightSide = new MotorControllerGroup(rightLeader, rightFollower);

  public final DifferentialDrive differentialDrive = new DifferentialDrive(leftSide, rightSide);

  // The left-side drive encoder
  // The gyro sensor
  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(DrivetrainConstants.PIGEON_CHANNEL);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightSide.setInverted(true);

    // Sets the distance per pulse for the encoders



    resetEncoders();
    zeroHeading();
      m_odometry =
        new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(m_pigeon.getYaw()), 
          ticksToMeters(leftLeader.getSelectedSensorPosition()), 
          ticksToMeters(leftLeader.getSelectedSensorPosition()));
  }



  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(m_pigeon.getYaw()), 
    ticksToMeters(leftLeader.getSelectedSensorPosition()), 
    ticksToMeters(leftLeader.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Pigeon Yaw", m_pigeon.getAngle());
    SmartDashboard.putNumber("Gyro Rate", m_pigeon.getRate());
    SmartDashboard.putNumber("Get Pose2D", Rotation2d.fromDegrees(m_pigeon.getYaw()).getDegrees());
    // System.out.println("Raw Rotation 2D: " + Rotation2d.fromDegrees(m_pigeon.getYaw()));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double ticksToMeters(double ticks) {
    return DrivetrainConstants.METERS_PER_TICK * ticks;
  }
  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(ticksToMeters(leftLeader.getSelectedSensorVelocity()) / 10, ticksToMeters(rightLeader.getSelectedSensorVelocity()) / 10);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(m_pigeon.getYaw()), 
      ticksToMeters(leftLeader.getSelectedSensorPosition()), 
      ticksToMeters(leftLeader.getSelectedSensorPosition()), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot, boolean square) {
    differentialDrive.arcadeDrive(fwd, rot, square);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (ticksToMeters(leftLeader.getSelectedSensorPosition()) + 
    ticksToMeters(leftLeader.getSelectedSensorPosition())) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  /*public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }*/

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  /*public Encoder getRightEncoder() {
    return m_rightEncoder;
  }*/

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_pigeon.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    //TODO: verify pigeon getYaw is compatible (check documentation)
    return m_pigeon.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  //TODO: find equivalent pigeon method (documentation)
  /*public double getTurnRate() {
    return -m_gyro.getRate();
  }*/







  public Command getAutonomousCommand(String fileName, boolean isFirstPath) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    /* var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DrivetrainConstants.ksVolts,
                DrivetrainConstants.kvVoltSecondsPerMeter,
                DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            DrivetrainConstants.kDriveKinematics,
            10); */

    // Follow specified path file
    PathPlannerTrajectory traj = 
    PathPlanner.loadPath(fileName, new PathConstraints(
      DrivetrainConstants.kMaxSpeedMetersPerSecond, 
      DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared));

      return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj,
            this::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(
                DrivetrainConstants.ksVolts,
                DrivetrainConstants.kvVoltSecondsPerMeter,
                DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            DrivetrainConstants.kDriveKinematics,
            this::getWheelSpeeds,
            new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
            new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            this::tankDriveVolts,
            false,
            this));
  }
}