// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.ArmUtils;

public class ChangeArmPosition extends CommandBase {
  ElevatorSubsystem arm;
  int setpoint;
  RobotContainer robotContainer;
  ArmUtils utils;
  
  /** Creates a new changeArmPosition. */
  public ChangeArmPosition(RobotContainer robotContainer, ArmUtils utils) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotContainer = robotContainer;
    arm = robotContainer.telescopingArm;
    this.utils = utils;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = utils.armSetpoint();
    arm.armPID(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
