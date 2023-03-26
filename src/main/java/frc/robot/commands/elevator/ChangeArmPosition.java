// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ChangeArmPosition extends CommandBase {
  ElevatorSubsystem arm;
  int setpoint;
  RobotContainer robotContainer;
  
  /** Creates a new changeArmPosition. */
  public ChangeArmPosition(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotContainer = robotContainer;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm = robotContainer.telescopingArm;
    if (robotContainer.gamePiece == RobotContainer.GamePieces.CONE) { // cone
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.LOW)
        setpoint = Constants.ArmConstants.CONE_LOW_GOAL_EXT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.MID)
        setpoint = Constants.ArmConstants.CONE_MID_GOAL_EXT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = Constants.ArmConstants.CONE_HIGH_GOAL_EXT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.SINGLE)
        setpoint = Constants.ArmConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if(robotContainer.gamePiece == RobotContainer.GamePieces.CUBE) {
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.LOW)
        setpoint = Constants.ArmConstants.CUBE_LOW_GOAL_EXT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.MID)
        setpoint = Constants.ArmConstants.CUBE_MID_GOAL_EXT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.HIGH)
        setpoint = Constants.ArmConstants.CUBE_HIGH_GOAL_EXT_TICKS;
      if(robotContainer.goalHeights == RobotContainer.GoalHeights.SINGLE)
        setpoint = Constants.ArmConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if (robotContainer.gamePiece == RobotContainer.GamePieces.STOW) { // stow
      setpoint = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.armPID(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isMotionCompleted();
  }
}
