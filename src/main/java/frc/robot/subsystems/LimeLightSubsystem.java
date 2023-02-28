// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubsystem extends SubsystemBase {
  NetworkTable limeLightTable; 
    NetworkTableEntry targetX; 
    NetworkTableEntry targetY; 
    NetworkTableEntry targetArea; 
    NetworkTableEntry targetFound;
    NetworkTableEntry ledMode;
    NetworkTableEntry botpose;

  /** Creates a new ExampleSubsystem. */
  public LimeLightSubsystem() {
    limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
    targetX = limeLightTable.getEntry("tx");
    targetY = limeLightTable.getEntry("ty");
    targetArea = limeLightTable.getEntry("ta");
    targetFound = limeLightTable.getEntry("tv");
    ledMode = limeLightTable.getEntry("ledMode");
    ledMode.setNumber(3);
    botpose = limeLightTable.getEntry("botpose");
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

  public void setLimeOn(boolean turnOn){
    limeLightTable.getEntry("ledMode").setNumber(turnOn ? 3 : 1);
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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    double x = targetX.getDouble(0.0);
    double y = targetY.getDouble(0.0);
    double area = targetArea.getDouble(0.0);
    double[] pose = botpose.getDoubleArray(new double[6]);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumberArray("LimelightTID", getTID().getDoubleArray(new double[6]));
    SmartDashboard.putBoolean("targetExists", foundTarget());
    SmartDashboard.putNumberArray("botpose", pose);
  }

  public NetworkTableEntry getTID() {
    return limeLightTable.getEntry("tid");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}