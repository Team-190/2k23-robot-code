// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PivotSubsystem;

public class ChangePivotPosition extends CommandBase {
  PivotSubsystem pivot;
  int setpoint;
  RobotContainer robotContainer;
  /** Creates a new changePivotPosition. */
  public ChangePivotPosition(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    pivot = robotContainer.pivot;
    
    this.robotContainer = robotContainer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(robotContainer.pivotDirection) {
    if (robotContainer.gamePiece == RobotContainer.GamePieces.CONE) {
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.LOW)
        setpoint = Constants.PivotConstants.CONE_LOW_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.MID)
        setpoint = Constants.PivotConstants.CONE_MID_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = Constants.PivotConstants.CONE_HIGH_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.SINGLE)
        setpoint = Constants.PivotConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
   else if (robotContainer.gamePiece == RobotContainer.GamePieces.CUBE) {
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.LOW)
        setpoint = Constants.PivotConstants.CUBE_LOW_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.MID)
        setpoint = Constants.PivotConstants.CUBE_MID_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = Constants.PivotConstants.CUBE_HIGH_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.SINGLE)
        setpoint = Constants.PivotConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if (robotContainer.gamePiece == RobotContainer.GamePieces.STOW) { // stow
      setpoint = 0;
    }
  } else {
    if (robotContainer.gamePiece == RobotContainer.GamePieces.CONE) {
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.LOW)
        setpoint = -1*Constants.PivotConstants.CONE_LOW_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.MID)
        setpoint = -1*Constants.PivotConstants.CONE_MID_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = -1*Constants.PivotConstants.CONE_HIGH_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.SINGLE)
        setpoint = -1*Constants.PivotConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
   else if (robotContainer.gamePiece == RobotContainer.GamePieces.CUBE) {
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.LOW)
        setpoint = -1*Constants.PivotConstants.CUBE_LOW_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.MID)
        setpoint = -1*Constants.PivotConstants.CUBE_MID_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = -1*Constants.PivotConstants.CUBE_HIGH_GOAL_PIVOT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = -1*Constants.PivotConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if (robotContainer.gamePiece == RobotContainer.GamePieces.STOW) { // stow
      setpoint = 0;
    }
  }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.pivotPID(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.isMotionCompleted();

  }
}
