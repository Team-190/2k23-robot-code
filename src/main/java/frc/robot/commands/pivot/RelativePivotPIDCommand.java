// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.utils.ArmUtils;
import frc.robot.utils.ArmUtils.PIVOT_DIRECTION;

public class RelativePivotPIDCommand extends CommandBase {

  PivotSubsystem pivotSubsystem;
  ArmUtils armUtils;
  DoubleSupplier relativeDegrees;
  double ticks;

  /** Creates a new RelativePivotPIDCommand. */
  public RelativePivotPIDCommand(RobotContainer robotContainer, DoubleSupplier relativeDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.

    pivotSubsystem = robotContainer.pivot;
    this.relativeDegrees = relativeDegrees;
    this.armUtils = robotContainer.armUtils;

    // addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ticks = relativeDegrees.getAsDouble() * PivotConstants.PIVOT_TICKS_PER_DEGREE;

    // If forwards then keep positive for up, else negate for up to go towards 0
    ticks = (armUtils.getPivotDirection() == PIVOT_DIRECTION.FORWARD ? ticks : -ticks) ;

    pivotSubsystem.pivotRelativePID(ticks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
