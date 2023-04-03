// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import javax.management.relation.RoleNotFoundException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.utils.input.AttackThree.AttackThreeAxis;

public class LimeLightAssist extends CommandBase {
  /** Creates a new LimeLightAssist. */
  DrivetrainSubsystem drivetrainSubsystem;
  LimeLightSubsystem limeLightSubsystem;
  RobotContainer container;
  public LimeLightAssist(RobotContainer container, DrivetrainSubsystem drivetrainSubsystem, LimeLightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    limeLightSubsystem = limelight;
    this.container = container;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLightSubsystem.setPipeline(2); // whatever is the object detection
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.arcadeDrive(container.leftStick.getAxis(AttackThreeAxis.Y), 
    Math.sin(limeLightSubsystem.getTX()), // check if this is degrees or radians and convert accordingly
    false); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.arcadeDrive(container.leftStick.getAxis(AttackThreeAxis.Y), 0, false); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !container.leftStick.getMiddleFaceButton();
  }
}
