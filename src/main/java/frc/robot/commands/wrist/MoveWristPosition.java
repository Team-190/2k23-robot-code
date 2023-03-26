// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;

public class MoveWristPosition extends CommandBase {
  WristSubsystem wrist;
  int setpoint;
  RobotContainer robotContainer;
  /** Creates a new moveWristPosition. */
  public MoveWristPosition(RobotContainer robotcontainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = robotcontainer.wrist;
    this.robotContainer = robotcontainer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(robotContainer.gamePiece == RobotContainer.GamePieces.CONE) {
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.LOW)
        setpoint = Constants.WristConstants.CONE_LOW_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.MID)
        setpoint = Constants.WristConstants.CONE_MID_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = Constants.WristConstants.CONE_HIGH_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.SINGLE)
        setpoint = Constants.WristConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if (robotContainer.gamePiece == RobotContainer.GamePieces.CUBE) {
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.LOW)
        setpoint = Constants.WristConstants.CUBE_LOW_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.MID)
        setpoint = Constants.WristConstants.CUBE_MID_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = Constants.WristConstants.CUBE_HIGH_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.SINGLE)
        setpoint = Constants.WristConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if (robotContainer.gamePiece == RobotContainer.GamePieces.STOW) { // stow
      setpoint = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.wristPID(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.isMotionCompleted();
  }
}
