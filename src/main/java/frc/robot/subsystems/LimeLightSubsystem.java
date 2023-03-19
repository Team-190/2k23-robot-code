// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubsystem extends SubsystemBase {
  NetworkTable limeLightTable; 
    public NetworkTableEntry targetX; 
    public NetworkTableEntry targetY; 
    public NetworkTableEntry targetV;
    NetworkTableEntry targetArea; 
    NetworkTableEntry targetFound;
    NetworkTableEntry ledMode;
    NetworkTableEntry pipeline;
    NetworkTableEntry botpose;
    NetworkTableEntry botpose_red;
    NetworkTableEntry botpose_blue;
    NetworkTableEntry ta;
    NetworkTableEntry camMode;
    public boolean enableVision = true;

  //read values periodically
  double x;
  double y;
  double v;
  double area;

  /** Creates a new ExampleSubsystem. */
  public LimeLightSubsystem() {
    limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
    targetX = limeLightTable.getEntry("tx");
    targetY = limeLightTable.getEntry("ty");
    targetV = limeLightTable.getEntry("tv");
    targetArea = limeLightTable.getEntry("ta");
    targetFound = limeLightTable.getEntry("tv");
    ledMode = limeLightTable.getEntry("ledMode");
    pipeline = limeLightTable.getEntry("pipeline");
    ledMode.setNumber(3);
    botpose = limeLightTable.getEntry("botpose");
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean foundTarget() {
    // Query some boolean state, such as a digital sensor.
    return limeLightTable.getEntry("tv").getBoolean(false);
  }

  public void setPipeline(int pipeLine) {
    this.pipeline.setNumber(pipeLine);
  }

  public double[] getAprilTagPose() {
    double[] pose = botpose.getDoubleArray(new double[6]);
    double[] xyYaw = new double[3];
    if (pose.length == 0) return null;
    if (targetArea.getDouble(0) < 0.4) {
      return null;
    }
    xyYaw[0] = pose[0];
    xyYaw[1] = pose[1];
    xyYaw[2] = pose[5];
    return xyYaw;
  }

  public double getTX(){
    return limeLightTable.getEntry("tx").getDouble(0.0);
  }
  
  public double getTY(){
    return limeLightTable.getEntry("ty").getDouble(0.0);
  }

  public double getTA(){
    return limeLightTable.getEntry("ta").getDouble(0.0);
  }

  public double[] translateBlue(double[] xyYaw) {
    double[] result = xyYaw;
    result[0] += 8.25;
    result[1] += 4.00;
    return result;
  }

  public double[] translateRed(double[] xyYaw) {
    double[] result = xyYaw;
    result[0] = 8.25 - result[0];
    result[1] += 4.00;
    result[2] += 180;
    return result;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    x = targetX.getDouble(0.0);
    y = targetY.getDouble(0.0);
    area = targetArea.getDouble(0.0);
    double[] pose = botpose.getDoubleArray(new double[6]);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumberArray("LimelightTID", getTID().getDoubleArray(new double[6]));
    SmartDashboard.putBoolean("targetExists", foundTarget());
    SmartDashboard.putNumberArray("botpose", pose);
    if (getAprilTagPose() == null) return;
    SmartDashboard.putNumberArray("botpose_blue", translateBlue(getAprilTagPose()));
    SmartDashboard.putNumberArray("botpose_red", translateRed(getAprilTagPose()));
    
    
  }

  public NetworkTableEntry getTID() {
    return limeLightTable.getEntry("tid");
  } 

  public boolean targetFound() {
    return v == 1.0;
  }

  public double degreesAskew(){
    return x;
  }

  public double getDistanceToTarget() {
    double limelightHeight = 22.5;
    double limelightMountAngleDegrees = 28; // Angle from horizontal
    double heightToGoal = 104.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees* (3.14159 / 180.0);

    double distance = (heightToGoal - limelightHeight) / Math.tan(angleToGoalRadians);
    return distance;

    //return (heightToGoal - limelightHeight)/Math.tan(Math.toRadians(limelightMountAngleDegrees + y));
  }

  public void toggleVision() {
    if (!getVision()) {
      lightOn();
    } else {
      lightOff();
    }
    enableVision = getVision();
    }

    public void setVision(boolean on) {
      ledMode.setNumber((on ? 3 : 1));
    }
    
    public void lightOff() {
      ledMode.setNumber(1);
    }


    public boolean getVision() {
      // return enableVision;
      return ledMode.getNumber(1).intValue() == 3;
    }
  public void lightOn() {
    ledMode.setNumber(3);
  }
  
  public void setCamMode(boolean mode) {
    camMode.setNumber((mode ? 1 : 0));
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}