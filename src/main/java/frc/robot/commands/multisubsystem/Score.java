// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multisubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Score extends SequentialCommandGroup {

  IntakeSubsystem claw;
  WristSubsystem wrist;
  PivotSubsystem pivot;
  ElevatorSubsystem elevator;
  RobotContainer robotContainer;

  /** Creates a new intakeCube. */
  public Score(RobotContainer robotcontainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotContainer = robotcontainer;
    claw = robotcontainer.intake;
    wrist = robotcontainer.wrist;
    pivot = robotcontainer.pivot;
    elevator = robotcontainer.telescopingArm;

    addRequirements(claw);

    if (robotContainer.gamePiece == RobotContainer.GamePieces.CONE){ // if it is a cone
      if (robotContainer.goalHeights == RobotContainer.GoalHeights.LOW) {
        pivot.pivotPID(Constants.PivotConstants.CONE_LOW_GOAL_PIVOT_TICKS);
        elevator.armPID(Constants.ArmConstants.CONE_LOW_GOAL_EXT_TICKS);
        wrist.wristPID(Constants.WristConstants.CONE_LOW_GOAL_WRIST_TICKS);
      }
      if (robotContainer.goalHeights == RobotContainer.GoalHeights.MID) {
        pivot.pivotPID(Constants.PivotConstants.CONE_MID_GOAL_PIVOT_TICKS);
        elevator.armPID(Constants.ArmConstants.CONE_MID_GOAL_EXT_TICKS);
        wrist.wristPID(Constants.WristConstants.CONE_MID_GOAL_WRIST_TICKS);
      }
      if (robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH) {
        pivot.pivotPID(Constants.PivotConstants.CONE_HIGH_GOAL_PIVOT_TICKS);
        elevator.armPID(Constants.ArmConstants.CONE_HIGH_GOAL_EXT_TICKS);
        wrist.wristPID(Constants.WristConstants.CONE_HIGH_GOAL_WRIST_TICKS);
      }
      
    }
    

  }
}
