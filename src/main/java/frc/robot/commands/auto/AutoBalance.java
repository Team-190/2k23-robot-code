// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private boolean onChargeStation = false;
  private boolean somewhatBalanced = false;
  private boolean balanced = false;
  private static final double TOLERANCE = 2.5;
  private static final double MEDIUM_SPEED = 0.4;
  private static final double SLOW_SPEED = 0.2;

  /** Creates a new AutoBalance. */
  public AutoBalance(DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    drivetrain.setBreakMode();
    onChargeStation = false;
    addRequirements(drivetrain);
    Timer timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (somewhatBalanced && Math.abs(drivetrain.getPitchDegrees()) < 10) {
      drivetrain.leftLeader.set(ControlMode.PercentOutput, -0.2);
      drivetrain.rightLeader.set(ControlMode.PercentOutput, -0.2);
      Timer.delay(1);
      drivetrain.leftLeader.set(ControlMode.PercentOutput, 0);
      drivetrain.rightLeader.set(ControlMode.PercentOutput, 0);
      Timer.delay(2);
      balanced = true;
    } else {
      drivetrain.leftLeader.set(ControlMode.PercentOutput, 0.5);
    drivetrain.rightLeader.set(ControlMode.PercentOutput, 0.5);

    if (drivetrain.getPitchDegrees() < -15) {
      drivetrain.leftLeader.set(ControlMode.PercentOutput, 0.5);
      drivetrain.rightLeader.set(ControlMode.PercentOutput, 0.5);
      onChargeStation = true;
    }
    if (onChargeStation && drivetrain.getPitchDegrees() > -8) {
      drivetrain.leftLeader.set(ControlMode.PercentOutput, 0.5);
      drivetrain.rightLeader.set(ControlMode.PercentOutput, 0.5);
      somewhatBalanced = true;
    }
    if (somewhatBalanced) {
      if (drivetrain.getPitchDegrees() > -1*TOLERANCE) {
        drivetrain.leftLeader.set(ControlMode.PercentOutput, 0.3);
        drivetrain.rightLeader.set(ControlMode.PercentOutput, 0.3);
      } else {
        drivetrain.leftLeader.set(ControlMode.PercentOutput, -0.3);
        drivetrain.rightLeader.set(ControlMode.PercentOutput, -0.3);
      }
    }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
