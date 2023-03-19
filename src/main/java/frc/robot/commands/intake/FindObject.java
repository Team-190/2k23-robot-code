// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;

public class FindObject extends CommandBase {
  public boolean objOnRight;
  public boolean objOnLeft;
  public boolean objAbove;
  public boolean objBelow;
  public final static double H_TOLERANCE = 2;
  public final static double V_TOLERANCE = 2;
  public LimeLightSubsystem limeLight;

  /** Creates a new FindCube. */
  public FindObject(LimeLightSubsystem limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limeLight.getTX() > H_TOLERANCE) {
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(limeLight.getTX()) <= H_TOLERANCE) {
      return true;
    }
    return false;
  }
}
