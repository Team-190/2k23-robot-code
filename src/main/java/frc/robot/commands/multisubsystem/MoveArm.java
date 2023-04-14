// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multisubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.ChangeArmPosition;
import frc.robot.commands.pivot.ChangePivotPosition;
import frc.robot.commands.wrist.ChangeWristPosition;
import frc.robot.utils.ArmUtils;
import frc.robot.utils.ArmUtils.ARM_STATE;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  RobotContainer container;
  ArmUtils utils;
  public MoveArm(RobotContainer container, ArmUtils utils) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.container = container;
    this.utils = utils;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (utils.elevator.armMotor.getSelectedSensorPosition() <= utils.armSetpoint() + Constants.ArmConstants.TOLERANCE
        && utils.getArmState() != ARM_STATE.STATION_SINGLE && utils.getArmState() != ARM_STATE.LOW) {
      (new SequentialCommandGroup(
          new ChangePivotPosition(container, utils),
          new ChangeArmPosition(container, utils),
          new ChangeWristPosition(container, utils),
          new InstantCommand(() -> container.moveArmFinished = true)
          )).schedule();
    } else {
      (new SequentialCommandGroup(
          new ChangeWristPosition(container, utils),
          new ChangeArmPosition(container, utils),
          new ChangePivotPosition(container, utils),
          new InstantCommand(() -> container.moveArmFinished = true)
          )).schedule();
      }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
