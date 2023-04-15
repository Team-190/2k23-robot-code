// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalanceV2 extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private boolean inclined = false;
  private static final double TOLERANCE = 2;
  private static final double maxSpeed = 0.2;
  private static final double maxPitch = 17;

  /** Creates a new AutoBalance. */
  public AutoBalanceV2(DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
 
    inclined = false;
    addRequirements(drivetrain);
    Timer timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setBreakMode();
    inclined = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (!inclined) {
      drivetrain.westCoastDrive(maxSpeed, maxSpeed, false);
      if (drivetrain.gyro.getPitch() > TOLERANCE) inclined = true;
    } else {
      double pitchPercent = -drivetrain.gyro.getPitch()/maxPitch;
      drivetrain.westCoastDrive(maxSpeed*pitchPercent*Math.abs(pitchPercent), maxSpeed*pitchPercent*Math.abs(pitchPercent), false);
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
