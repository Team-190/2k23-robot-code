package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends PIDSubsystem {

    public final WPI_TalonFX leftLeader = new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER_CHANNEL);
    private final WPI_TalonFX leftFollower =
            new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER_CHANNEL);

    public final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER_CHANNEL);
    private final WPI_TalonFX rightFollower =
            new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER_CHANNEL);

    private final MotorControllerGroup leftSide = new MotorControllerGroup(leftLeader, leftFollower);
    private final MotorControllerGroup rightSide = new MotorControllerGroup(rightLeader, rightFollower);

    public final DifferentialDrive differentialDrive = new DifferentialDrive(leftSide, rightSide);

    // Objects for PID tracking
    // private final AHRS navx = new AHRS(SPI.Port.kMXP);
    public final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    private final DifferentialDriveOdometry odometry =
            new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0 , 0);
    private double angleOffset = 0;

    /**
    * Construct an instance of the Drivetrain
    *
    * @param kP The P value for the PIDF
    * @param ki The I value for the PIDF
    * @param kD The D value for the PIDF
    */
    public DrivetrainSubsystem(double kP, double ki, double kD) {

        super(new PIDController(kP, ki, kD));

        // Reset configuration to defaults
        leftLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        // Configure the Followers
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // Configure invert type on the motors
        leftLeader.setInverted(true);
        leftFollower.setInverted(true);
        rightLeader.setInverted(false);
        rightFollower.setInverted(false);

        leftFollower.setStatusFramePeriod(1, 255);
        rightFollower.setStatusFramePeriod(1, 255);
        leftFollower.setStatusFramePeriod(2, 255);
        rightFollower.setStatusFramePeriod(2, 255);

        // Set Break Mode
        setBreakMode();
        // setCoastMode();

        // Configure the PID feedback and constants
        leftLeader.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                DrivetrainConstants.PID_LOOPTYPE,
                DrivetrainConstants.TIMEOUT_MS);
        rightLeader.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                DrivetrainConstants.PID_LOOPTYPE,
                DrivetrainConstants.TIMEOUT_MS);

        configPIDF(
                leftLeader,
                DrivetrainConstants.P,
                DrivetrainConstants.I,
                DrivetrainConstants.D,
                DrivetrainConstants.F);
        configPIDF(
                rightLeader,
                DrivetrainConstants.P,
                DrivetrainConstants.I,
                DrivetrainConstants.D,
                DrivetrainConstants.F);

        // Wait for Gyro init before finishing DriveSubsystem init
        try {
            Thread.sleep(2000);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Reset Drive Odometry, Encoders, and Gyro
        resetAll();
        setSetpoint(0);
    }

    @Override
    public void periodic() {
        
        // SmartDashboard.putNumber("Left Drive Encoder", leftLeader.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Left Follower Drive Encoder", leftFollower.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Right Drive Encoder", rightLeader.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Right Follower Drive Encoder", rightFollower.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Difference Meters", Math.abs(getDistanceMeters(leftLeader)-getDistanceMeters(rightLeader)));
        // SmartDashboard.putNumber("Get left wheel speed", leftLeader.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Get right wheel speed", rightLeader.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("gyro raw yaw", gyro.getAngle());
        // SmartDashboard.putNumber("gyro yaw", getYawDegrees());
        // SmartDashboard.putNumber("Meters Left Side Traveled", getDistanceMeters(leftLeader));
        // SmartDashboard.putNumber("Meters Right Side Traveled", getDistanceMeters(rightLeader));
        
        

        // Update the Odometry
        odometry.update(
            Rotation2d.fromDegrees(getYawDegrees()),
            getDistanceMeters(leftLeader),
            getDistanceMeters(rightLeader)
        );
        
        
    }

    /**
     * Gets distance in meters
     *
     * @return the distance in meters
     */
    public double getDistanceMeters(TalonFX talon) {
        return talon.getSelectedSensorPosition() * DrivetrainConstants.METERS_PER_COUNT;
    }

    /**
     * (counts / 100 ms) * (meters / count) * (10 ms / 1 s) == (meters / second)
     *
     * @return Wheel speeds in meters / second
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftLeader.getSelectedSensorVelocity() * DrivetrainConstants.METERS_PER_COUNT * 10,
            rightLeader.getSelectedSensorVelocity() * DrivetrainConstants.METERS_PER_COUNT * 10
        );
    }

    

    /**
     * Gets the chassis's yaw (orientation of the robot)
     *
     * @return yaw in degrees (-180 to 180 degrees)
     */
    public double getYawDegrees() {
        double angle = ((Math.abs(gyro.getAngle())) + angleOffset) % 360;
        if (angle <= 180.0)
            return -angle;
        return -(angle - 360);
    }
    
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(isFirstPath){
                  this.resetOdometry(traj.getInitialPose());
              }
            }),
            new PPRamseteCommand(
                traj, 
                this::getPose, // Pose supplier
                new RamseteController(),
                DrivetrainConstants.DRIVE_FEED_FORWARD,
                DrivetrainConstants.DRIVE_KINEMATICS, // DifferentialDriveKinematics
                this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                this::tankDriveVolts, // Voltage biconsumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            )
        );
    }

    /**
     * Sets drive motors to brake
     */
    public void setBreakMode() {
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets drive motors to coast
     */
    public void setCoastMode() {
        leftLeader.setNeutralMode(NeutralMode.Coast);
        rightLeader.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets encoders to 0
     */
    private void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
        leftFollower.setSelectedSensorPosition(0);
        rightFollower.setSelectedSensorPosition(0);
    }


    /**
     * Resets the Yaw position of the gyro and sets an offset
     * @param forwards True if robot is going forwards, false if backwards
     */
    public void resetGyro(boolean forwards) {
        gyro.reset();
        angleOffset = forwards ? 0 : 180; // No offset if true, 180 offset if false
    }


    /**
     * Resets odometry to specified pose
     *
     * @param pose pose to reset to
     */
    private void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(Rotation2d.fromDegrees(getYawDegrees()), 0 , 0, pose);
    }

    /**
     * Resets gyro, Encoders, odometry
     */
    public void resetAll() {
        resetGyro(true);
        resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(getYawDegrees())));
    }

    /**
    * Configures PIDF, not used by Trajectories
    *
    * @param motorController The motor controller to configure
    * @param P proportional value
    * @param I integral value
    * @param D derivative value
    * @param F feed forward value
    */
    public void configPIDF(WPI_TalonFX motorController, double P, double I, double D, double F) {
        motorController.config_kP(DrivetrainConstants.SLOT_ID, P);
        motorController.config_kI(DrivetrainConstants.SLOT_ID, I);
        motorController.config_kD(DrivetrainConstants.SLOT_ID, D);
        motorController.config_kF(DrivetrainConstants.SLOT_ID, F);
    }

    /**
    * Drive in Arcade mode
    *
    * @param throttle The Speed to go at
    * @param rotation The Rotation rate
    * @param square Whether to square the inputs
    */
    public void arcadeDrive(double throttle, double rotation, boolean square) {
        differentialDrive.arcadeDrive(throttle, rotation, square);
    }

    /**
    * Drive in WestCoast (Tank Drive mode)
    *
    * @param leftStick The Speed of the left side of the robot
    * @param rightStick The Speed of the right side of the robot
    * @param square Whether to square the inputs
    */
    public void westCoastDrive(double leftStick, double rightStick, boolean square) {
        //differentialDrive.tankDrive(Math.copySign(Math.pow(leftStick, power), leftStick), Math.copySign(Math.pow(leftStick, power), leftStick));
       differentialDrive.tankDrive(leftStick, rightStick, square);
    }

    /**
    * Drive in Curvature Drive (Ask Zach Boyer for an explanation)
    *
    * @param throttle The Speed to go at
    * @param radius The turning radius
    * @param quickTurn True to enable turn-in-place, False to disable
    */
    public void curvatureDrive(double throttle, double radius, boolean quickTurn) {
        differentialDrive.curvatureDrive(throttle, radius, quickTurn);
    }

    /**
     * Gets the magnitude of the velocity of the robot
     * @return the magnitude of the drivetrain
     */
    public double magnitudeVelocity() {
        double leftSpeeds = leftLeader.getSelectedSensorVelocity() * DrivetrainConstants.METERS_PER_COUNT * 10;
        double rightSpeeds = rightLeader.getSelectedSensorVelocity() * DrivetrainConstants.METERS_PER_COUNT * 10;

        // Distance from the left wheel to the center of the instantanious turn
        double lengthToVirtualCenter = (leftSpeeds * DrivetrainConstants.TRACKWIDTH_METERS) / (rightSpeeds - leftSpeeds);

        // The robot's angular velocity
        double angularVelocity = leftSpeeds/lengthToVirtualCenter;

        // The instantanious velocity of the shooter
        double magnitudeVelocity = angularVelocity * (lengthToVirtualCenter + (DrivetrainConstants.TRACKWIDTH_METERS/2));

        return magnitudeVelocity;
    }

    // PID methods
    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        leftFollower.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
        rightFollower.setVoltage(rightVolts);
        differentialDrive.feed();
    }

    @Override
    protected void useOutput(double output, double setpoint) {}

    @Override
    protected double getMeasurement() {
        return 0;
    }


    /**
    * Get the position of the robot relative to the starting position
    *
    * @return the position of the robot
    */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
}
