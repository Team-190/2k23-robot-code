// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multisubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class MoveArmDeadline extends CommandBase {
  /** Creates a new MoveArmDeadline. */
  RobotContainer container;
  public MoveArmDeadline(RobotContainer container) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.container = container;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    container.moveArmFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    container.moveArmFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return container.moveArmFinished;
  }
}
