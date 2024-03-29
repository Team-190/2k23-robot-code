// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SimpleAuto extends CommandBase {
  DrivetrainSubsystem drivetrain;
  /** Creates a new SimpleAuto. */
  public SimpleAuto(DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.leftLeader.set(ControlMode.PercentOutput, -0.5);
    drivetrain.rightLeader.set(ControlMode.PercentOutput, -0.5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.leftLeader.set(ControlMode.PercentOutput, -0.5);
    drivetrain.rightLeader.set(ControlMode.PercentOutput, -0.5);

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
