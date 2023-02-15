// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  public enum ledMode{
    ON,
    OFF,
    BLINK
  };

  double[] limelightPosition = {1,1,1};

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    
  }

  public void setLedMode(ledMode mode){
    switch(mode) {
      case OFF:
        LimelightHelpers.setLEDMode_ForceOff("");
        break;
      case ON:
        LimelightHelpers.setLEDMode_ForceOff("");
        break;
      case BLINK:
        LimelightHelpers.setLEDMode_ForceOff("");
        break;
    }
  }

  // public void setLimelightPipeline(ledPipeline pipeline){
  //   switch(pipeline){
  //     case RETRO:
  //       LimelightHelpers.
  //   }
  // }

  public Pose2d getRobotPosition(){
    LimelightHelpers.LimelightTarget_Fiducial test = new LimelightHelpers.LimelightTarget_Fiducial();
    return test.getRobotPose_FieldSpace2D();
  }

  // LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
  public LimelightHelpers.LimelightResults getLimelightResults(){
    return LimelightHelpers.getLatestResults("");
  }

  // public LimelightHelpers.LimelightTarget_Fiducial getAprilTags(){
  //   return LimelightHelpers.getLatestResults("").targetingResults.targets_Fiducials;
  // }
  // llresults.results.targets_Fiducials;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
