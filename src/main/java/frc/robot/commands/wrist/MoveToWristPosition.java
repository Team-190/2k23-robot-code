// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class MoveToWristPosition extends CommandBase {
  WristSubsystem wrist;
  int setpoint;
  RobotContainer robotcontainer;
  /** Creates a new moveToPivotPosition. */
  public MoveToWristPosition(RobotContainer robotcontainer, int setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotcontainer = robotcontainer;
    wrist = robotcontainer.wrist;
    this.setpoint = setpoint;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.wristPID(setpoint);;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pivot.pivotPID(setpoint);
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
