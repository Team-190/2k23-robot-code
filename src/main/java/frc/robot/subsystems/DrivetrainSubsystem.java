package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoTurn;
import frc.robot.commands.auto.SimpleAuto;

public class DrivetrainSubsystem extends PIDSubsystem {

    public final WPI_TalonFX leftLeader = new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER_CHANNEL, InputConstants.CANIVORE_BUS_NAME);
    private final WPI_TalonFX leftFollower =
            new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER_CHANNEL, InputConstants.CANIVORE_BUS_NAME);

    public final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER_CHANNEL, InputConstants.CANIVORE_BUS_NAME);
    private final WPI_TalonFX rightFollower =
            new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER_CHANNEL, InputConstants.CANIVORE_BUS_NAME);

    public final MotorControllerGroup leftSide = new MotorControllerGroup(leftLeader, leftFollower);
    public final MotorControllerGroup rightSide = new MotorControllerGroup(rightLeader, rightFollower);

    public final DifferentialDrive differentialDrive = new DifferentialDrive(leftSide, rightSide);

    // Objects for PID tracking
    // private final AHRS navx = new AHRS(SPI.Port.kMXP);
    public final Pigeon2 gyro = new Pigeon2(SensorConstants.GYRO_CHANNEL, InputConstants.CANIVORE_BUS_NAME);
    private DifferentialDriveOdometry odometry =
            new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0 , 0);
    private double angleOffset = 0;

    private LimeLightSubsystem limeLight;
    private RobotContainer robotContainer;

    NeutralMode currenNeutralMode = NeutralMode.Brake;
    
    // public final AHRS navx = new AHRS(SPI.Port.kMXP);



    /**
    * Construct an instance of the Drivetrain
    *
    * @param kP The P value for the PIDF
    * @param ki The I value for the PIDF
    * @param kD The D value for the PIDF
    */
    public DrivetrainSubsystem(double kP, double ki, double kD, RobotContainer container) {

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
        //setCoastMode();

        try {
            Thread.sleep(2000);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Reset Drive Odometry, Encoders, and Gyro
        resetAll();
        setSetpoint(0);

        SmartDashboard.putData("AutoBalance", new AutoBalance(this));
        SmartDashboard.putData("SimpleAuto", new SimpleAuto(this));
        SmartDashboard.putData("AutoTurn", new AutoTurn(this));

        robotContainer = container;
        limeLight = robotContainer.limeLightSubsystem;

        odometry =
         new DifferentialDriveOdometry(
             Rotation2d.fromDegrees(gyro.getYaw()), getDistanceMeters(leftLeader),
             getDistanceMeters(rightLeader));
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
        SmartDashboard.putNumber("gyro raw yaw", gyro.getYaw());
        SmartDashboard.putNumber("Get Average Distance Meters", getAverageDistanceMeters());
        // SmartDashboard.putNumber("gyro yaw", getYawDegrees());
        // SmartDashboard.putNumber("Meters Left Side Traveled", getDistanceMeters(leftLeader));
        // SmartDashboard.putNumber("Meters Right Side Traveled", getDistanceMeters(rightLeader));
        SmartDashboard.putNumber("Pitch", getPitchDegrees());
        SmartDashboard.putString("NeutralMode", getNeutralMode().name());
        
        

        // Update the Odometry
        odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw()),
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
     * Gets the average distance of the motors
     * @return
     */
    public double getAverageDistanceMeters() {
        return ((leftLeader.getSelectedSensorPosition() + rightLeader.getSelectedSensorPosition()) * DrivetrainConstants.METERS_PER_COUNT)/2;
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
        double angle = ((Math.abs(gyro.getYaw())) + angleOffset) % 360;
        if (angle <= 180.0)
            return -angle;
        return -(angle - 360);
    }

    public double getPitchDegrees() {
        return gyro.getPitch();
    }

    public NeutralMode getNeutralMode() {
        return currenNeutralMode;
    }

    /**
     * Sets drive motors to brake
     */
    public void setBreakMode() {
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
        currenNeutralMode = NeutralMode.Brake;
    }

    /**
     * Sets drive motors to coast
     */
    public void setCoastMode() {
        leftLeader.setNeutralMode(NeutralMode.Coast);
        rightLeader.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
        currenNeutralMode = NeutralMode.Coast;
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
        gyro.setYaw(0);
        angleOffset = forwards ? 0 : 180; // No offset if true, 180 offset if false
    }


    /**
     * Resets odometry to specified pose
     *
     * @param pose pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), 0 , 0, pose);
    }

    public void setOdometryAprilTag() {
        double[] xyYaw = limeLight.getAprilTagPose();
        if (xyYaw == null) return;
        xyYaw = limeLight.translateBlue(xyYaw);
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), 
        getDistanceMeters(leftLeader), getDistanceMeters(rightLeader), 
        new Pose2d(new Translation2d(xyYaw[0], xyYaw[1]), Rotation2d.fromDegrees(gyro.getYaw())));
        
    }

    /**
     * Resets gyro, Encoders, odometry
     */
    public void resetAll() {
        resetGyro(true);
        resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(gyro.getYaw())));
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
    /*public void configPIDF(WPI_TalonFX motorController, double P, double I, double D, double F) {
        motorController.config_kP(DrivetrainConstants.SLOT_ID, P);
        motorController.config_kI(DrivetrainConstants.SLOT_ID, I);
        motorController.config_kD(DrivetrainConstants.SLOT_ID, D);
        motorController.config_kF(DrivetrainConstants.SLOT_ID, F);
    }*/

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
        // differentialDrive.tankDrive(Math.copySign(Math.pow(leftStick, power), leftStick), Math.copySign(Math.pow(leftStick, power), leftStick));

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

    /**
     * For charge station balancing -- drives until the gyro output is within the tolerance
     * @param leftVelocity between 0 and 1
     * @param rightVelocity between 0 and 1
     * @param tolerance in degrees
     */
    public void driveUntil(double leftVelocity, double rightVelocity, double tolerance) {

    }

    public void driveWithVision(double leftInput, double rightInput, boolean squared){
        double tx = limeLight.degreesAskew();
        double kp = 0.015; // 0.05;
        double minTurn = 0.08;
        double steering_adjust = 0.0;
        boolean targetFound = limeLight.targetFound();
        if(!targetFound)
            steering_adjust=0;
        else{
            if(Math.abs(tx)>1){
                if(tx < 0){
                    steering_adjust = kp * tx;
                } else {
                    steering_adjust = kp * tx;
                }
            }
        }
        if (squared) {
            leftInput = Math.copySign(Math.pow(leftInput, 2), leftInput);
            rightInput = Math.copySign(Math.pow(rightInput, 2), rightInput);
        }
        
        this.westCoastDrive(leftInput + minThresholdSignedValue(steering_adjust, .5), rightInput + -minThresholdSignedValue(steering_adjust, .5), false);
    }

    public void seekTarget(){
        double tx = limeLight.degreesAskew();
        double kp = 0.01; // 0.05;
        double minTurn = 0.08;
        double steering_adjust = 0.0;
        boolean targetFound = limeLight.targetFound();
        if(!targetFound)
            steering_adjust=0;
        else{
            if(Math.abs(tx)>1){
                if(tx < 0){
                    // steering_adjust = kp * tx - minTurn;
                    steering_adjust = kp * tx;
                } else {
                    // steering_adjust = kp * tx + minTurn;
                    steering_adjust = kp * tx;
                }
            }
        }
        // leftLeader.set(ControlMode.PercentOutput, minThresholdSignedValue(steering_adjust, .5));
        // rightLeader.set(ControlMode.PercentOutput, -minThresholdSignedValue(steering_adjust, .5));
        // this.westCoastDrive(steering_adjust, -steering_adjust, false);
        this.westCoastDrive(minThresholdSignedValue(steering_adjust, .5), -minThresholdSignedValue(steering_adjust, .5), false);
    }


    public void goToDistance(double distance, LimeLightSubsystem limeLightSubsystem) {
        double kDist = 0.01;
        double currentDistance = limeLightSubsystem.getAprilTagDistance();
        double desiredDistance = distance;
        double distanceError = currentDistance -desiredDistance;
        double drivingAdjust = minThresholdSignedValue(kDist * distanceError, .5);
        SmartDashboard.putNumber("drivingAdjust", drivingAdjust);
        westCoastDrive(drivingAdjust, drivingAdjust, false);
    }

    public void goAim(double distance, double rotThreshold, double distThreshold, LimeLightSubsystem limeLightSubsystem){
        double kDist = 0.01;
        double kp = 0.01;
        double minTurn = 0.05;
        double minDist = 0.07;
        double tx = limeLightSubsystem.degreesAskew();
        double distance_adjust = 0.0;
        double distanceError = limeLightSubsystem.getAprilTagDistance() - distance;
        if(Math.abs(distanceError) > distThreshold){
            if(distanceError<0){
                distance_adjust = kDist * distanceError - minDist;
            } else {
                distance_adjust = kDist * distanceError + minDist;
            }
        }
        double steering_adjust = 0;
        if(Math.abs(tx)>rotThreshold){
            if(tx < 0){
                steering_adjust = kp * tx - minTurn;
            } else {
                steering_adjust = kp * tx + minTurn;
            }
        }
        distance_adjust = minThresholdSignedValue(distance_adjust, .5);
        westCoastDrive(steering_adjust+distance_adjust, -steering_adjust+distance_adjust, false);
    }

    private double minThresholdSignedValue(double value, double threshold){
        return Math.signum(value) * Math.min(Math.abs(value), threshold);
    }
}
