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

public class AutoBalance extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private boolean onChargeStation = false;
  private boolean somewhatBalanced = false;
  private boolean balanced = false;
  private static final double TOLERANCE = 10;
  private static final double MEDIUM_SPEED = 0.4;
  private static final double SLOW_SPEED = 0.2;

  /** Creates a new AutoBalance. */
  public AutoBalance(DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
 
    onChargeStation = false;
    addRequirements(drivetrain);
    Timer timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setBreakMode();
    onChargeStation = false;
    somewhatBalanced = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!(somewhatBalanced && onChargeStation)) {
      // drivetrain.leftSide.set( 0.3);
      // drivetrain.rightSide.set( -0.3);
      drivetrain.westCoastDrive(0.27, 0.27, false);
    }
    if (drivetrain.getPitchDegrees() < -20 && !onChargeStation){
        drivetrain.westCoastDrive(0.2, 0.2, false);
        onChargeStation = true;

    } else if (drivetrain.getPitchDegrees() > -13 && !somewhatBalanced && onChargeStation) {
      drivetrain.westCoastDrive(-0.3, -0.3, false);
      Timer.delay(0.15);
      somewhatBalanced = true;
    } else if (somewhatBalanced) {
      if (drivetrain.getPitchDegrees() < -1*TOLERANCE) {
        drivetrain.westCoastDrive(0.14, 0.14, false);
      } else if (drivetrain.getPitchDegrees() > TOLERANCE) {
        drivetrain.westCoastDrive(-0.15, -0.15, false);
      } else {
        drivetrain.westCoastDrive(0.0, 0.0, false);
      }
    }

    // if (somewhatBalanced && Math.abs(drivetrain.getPitchDegrees()) < 5) {
      // drivetrain.leftSide.set( -0.3);
      // drivetrain.rightSide.set( -0.3);
    //   // drivetrain.leftFollower.set(ControlMode.PercentOutput, -0.3);
    //   // drivetrain.rightLeader.set(ControlMode.PercentOutput, -0.3);
    //   // drivetrain.rightFollower.set(ControlMode.PercentOutput, -0.3);
    //   Timer.delay(1);
    //   // drivetrain.leftLeader.set(ControlMode.PercentOutput, 0);
    //   // drivetrain.rightLeader.set(ControlMode.PercentOutput, 0);
    //   drivetrain.leftSide.set(0);
    //   drivetrain.rightSide.set(0);
    //   Timer.delay(2);
    //   balanced = true;
    // } else {
    //   // drivetrain.leftLeader.set(ControlMode.PercentOutput, 0.5);
    //   // drivetrain.rightLeader.set(ControlMode.PercentOutput, 0.5);
    //   if (!onChargeStation) {
    //     drivetrain.leftSide.set( 0.2);
    //     drivetrain.rightSide.set( 0.2);
    //   }
      

    // if (drivetrain.getPitchDegrees() < -15 && !onChargeStation) {
    //   // drivetrain.leftLeader.set(ControlMode.PercentOutput, 0.3);
    //   // drivetrain.rightLeader.set(ControlMode.PercentOutput, 0.3);
    //   drivetrain.leftSide.set( 0.2);
    //   drivetrain.rightSide.set(0.2);
    //   onChargeStation = true;
    // }
    // if (onChargeStation && drivetrain.getPitchDegrees() > -8 && !somewhatBalanced) {
    //   // drivetrain.leftLeader.set(ControlMode.PercentOutput, 0.3);
    //   // drivetrain.rightLeader.set(ControlMode.PercentOutput, 0.3);
    //   drivetrain.leftSide.set( 0.1);
    //   drivetrain.rightSide.set(0.1);
    //   somewhatBalanced = true;
    // }
    // if (somewhatBalanced) {
      // if (drivetrain.getPitchDegrees() > -1*TOLERANCE) {
      //   // drivetrain.leftLeader.set(ControlMode.PercentOutput, 0.3);
      //   // drivetrain.rightLeader.set(ControlMode.PercentOutput, 0.3);
      //   drivetrain.leftSide.set( 0.2);
      //   drivetrain.rightSide.set(0.2);
      // } else {
      //   // drivetrain.leftLeader.set(ControlMode.PercentOutput, -0.3);
      //   // drivetrain.rightLeader.set(ControlMode.PercentOutput, -0.3);
      //   drivetrain.leftSide.set( -0.2);
      //   drivetrain.rightSide.set( -0.2);
      // }
    // }
    // }
    
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
