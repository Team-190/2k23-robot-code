// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.ArmUtils;

public class ChangeWristPosition extends CommandBase {
  WristSubsystem wrist;
  int setpoint;
  RobotContainer robotContainer;
  ArmUtils utils;
  /** Creates a new moveWristPosition. */
  public ChangeWristPosition(RobotContainer robotcontainer, ArmUtils utils) {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = robotcontainer.wrist;
    this.utils = utils;
    this.robotContainer = robotcontainer;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = utils.wristSetpoint();
    wrist.wristPID(setpoint);
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
    return wrist.isMotionCompleted();
  }
}
