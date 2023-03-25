// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PivotSubsystem;

public class moveToPivotPosition extends CommandBase {
  PivotSubsystem pivot;
  int setpoint;
  /** Creates a new moveToPivotPosition. */
  public moveToPivotPosition(RobotContainer robotcontainer, int setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    pivot = robotcontainer.pivot;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
