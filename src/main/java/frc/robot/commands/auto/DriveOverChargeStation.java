// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveOverChargeStation extends CommandBase {
  /** Creates a new DriveOverChargeStation. */
  boolean declined;
  RobotContainer container;
  double speed = 0.4;
  double pitchTolerance = 2;
  boolean reverse;
  DrivetrainSubsystem drivetrain;
  public DriveOverChargeStation(RobotContainer robotContainer, boolean reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    container = robotContainer;
    drivetrain = robotContainer.drivetrainSubsystem;
    declined = false;
    this.reverse = reverse;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    declined = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!reverse) {
      drivetrain.westCoastDrive(speed,speed, false);
      if (drivetrain.gyro.getPitch() > pitchTolerance) declined = true;
    } else {
      drivetrain.westCoastDrive(-speed,-speed, false);
      if (drivetrain.gyro.getPitch() < -pitchTolerance) declined = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.westCoastDrive(0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (declined && Math.abs(container.drivetrainSubsystem.gyro.getPitch()) < pitchTolerance);
  }
}
