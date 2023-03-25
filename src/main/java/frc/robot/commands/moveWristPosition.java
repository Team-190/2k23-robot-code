// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;

public class moveWristPosition extends CommandBase {
  WristSubsystem wrist;
  int setpoint;
  RobotContainer robotContainer;
  /** Creates a new moveWristPosition. */
  public moveWristPosition(RobotContainer robotcontainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = robotcontainer.wrist;
    this.robotContainer = robotcontainer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(robotContainer.gamePiece == 1) {
      if(robotContainer.goalHeight == 0)
        setpoint = Constants.WristConstants.CONE_LOW_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeight == 1)
        setpoint = Constants.WristConstants.CONE_MID_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeight == 2)
        setpoint = Constants.WristConstants.CONE_HIGH_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeight == 3)
        setpoint = Constants.WristConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if (robotContainer.gamePiece == 0) {
      if(robotContainer.goalHeight == 0)
        setpoint = Constants.WristConstants.CUBE_LOW_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeight == 1)
        setpoint = Constants.WristConstants.CUBE_MID_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeight == 2)
        setpoint = Constants.WristConstants.CUBE_HIGH_GOAL_WRIST_TICKS;
      if(robotContainer.goalHeight == 3)
        setpoint = Constants.WristConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if (robotContainer.gamePiece == 2) { // stow
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
