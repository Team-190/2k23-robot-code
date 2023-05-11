// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveWithVision extends CommandBase {

  DrivetrainSubsystem drivetrainSubsystem;
  BooleanSupplier trackTarget;
  DoubleSupplier leftInput;
  DoubleSupplier rightInput;
  boolean squared;

  /** Creates a new DriveWithVision. */
  public DriveWithVision(RobotContainer robotContainer, DoubleSupplier leftInput, DoubleSupplier rightInput, boolean squared, BooleanSupplier trackTarget) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSubsystem = robotContainer.drivetrainSubsystem;
    this.trackTarget = trackTarget;
    this.leftInput = leftInput;
    this.rightInput = rightInput;
    this.squared = squared;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (trackTarget.getAsBoolean()) {
      drivetrainSubsystem.driveWithVision(leftInput.getAsDouble(), rightInput.getAsDouble(), squared);
    } else {
      drivetrainSubsystem.westCoastDrive(leftInput.getAsDouble(), rightInput.getAsDouble(), squared);
    }

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
