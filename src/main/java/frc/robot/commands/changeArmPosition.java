// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TelescopingArm;

public class changeArmPosition extends CommandBase {
  TelescopingArm arm;
  int setpoint;
  RobotContainer robotContainer;
  
  /** Creates a new changeArmPosition. */
  public changeArmPosition(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotContainer = robotContainer;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm = robotContainer.telescopingArm;
    if (robotContainer.gamePiece == 1) { // cone
      if(robotContainer.goalHeight == 0)
        setpoint = Constants.ArmConstants.CONE_LOW_GOAL_EXT_TICKS;
      if(robotContainer.goalHeight == 1)
        setpoint = Constants.ArmConstants.CONE_MID_GOAL_EXT_TICKS;
      if(robotContainer.goalHeight == 2)
        setpoint = Constants.ArmConstants.CONE_HIGH_GOAL_EXT_TICKS;
      if(robotContainer.goalHeight == 3)
        setpoint = Constants.ArmConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if(robotContainer.gamePiece == 0) {
      if(robotContainer.goalHeight == 0)
        setpoint = Constants.ArmConstants.CUBE_LOW_GOAL_EXT_TICKS;
      if(robotContainer.goalHeight == 1)
        setpoint = Constants.ArmConstants.CUBE_MID_GOAL_EXT_TICKS;
      if(robotContainer.goalHeight == 2)
        setpoint = Constants.ArmConstants.CUBE_HIGH_GOAL_EXT_TICKS;
      if(robotContainer.goalHeight == 3)
        setpoint = Constants.ArmConstants.SINGLE_PICKUP_PIVOT_TICKS;
    }
    else if (robotContainer.gamePiece == 2) { // stow
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
