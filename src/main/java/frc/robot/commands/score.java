// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.WristSubsystem;

public class score extends SequentialCommandGroup {

  IntakeSubsystem claw;
  WristSubsystem wrist;
  PivotSubsystem pivot;
  TelescopingArm elevator;

  /** Creates a new intakeCube. */
  public score(RobotContainer robotcontainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    claw = robotcontainer.claw;
    wrist = robotcontainer.wrist;
    pivot = robotcontainer.pivot;
    elevator = robotcontainer.telescopingArm;

    addRequirements(claw);

    if (robotcontainer.gamePiece == 1){ // if it is a cone
      if (robotcontainer.goalHeight == 0) {
        pivot.pivotPID(Constants.PivotConstants.CONE_LOW_GOAL_PIVOT_TICKS);
        elevator.armPID(Constants.ArmConstants.CONE_LOW_GOAL_EXT_TICKS);
        wrist.wristPID(Constants.WristConstants.CONE_LOW_GOAL_WRIST_TICKS);
      }
      if (robotcontainer.goalHeight == 1) {
        pivot.pivotPID(Constants.PivotConstants.CONE_MID_GOAL_PIVOT_TICKS);
        elevator.armPID(Constants.ArmConstants.CONE_MID_GOAL_EXT_TICKS);
        wrist.wristPID(Constants.WristConstants.CONE_MID_GOAL_WRIST_TICKS);
      }
      if (robotcontainer.goalHeight == 2) {
        pivot.pivotPID(Constants.PivotConstants.CONE_HIGH_GOAL_PIVOT_TICKS);
        elevator.armPID(Constants.ArmConstants.CONE_HIGH_GOAL_EXT_TICKS);
        wrist.wristPID(Constants.WristConstants.CONE_HIGH_GOAL_WRIST_TICKS);
      }
      
    }
    

  }
}
