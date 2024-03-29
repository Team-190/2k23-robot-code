// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCone extends CommandBase {

  IntakeSubsystem clawsubsystem;
  boolean aboveLimit;
  boolean belowLimit;
  /** Creates a new intakeCube. */
  public IntakeCone(RobotContainer robotcontainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    clawsubsystem = robotcontainer.intake;
    addRequirements(clawsubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aboveLimit = false;
    belowLimit = false;
    clawsubsystem.intake();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(clawsubsystem.clawMotor.getStatorCurrent())>40) {
      aboveLimit = true;
    }
    if (aboveLimit && Math.abs(clawsubsystem.clawMotor.getStatorCurrent())<30) {
      belowLimit = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawsubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(clawsubsystem.clawMotor.getStatorCurrent()) > 50 && belowLimit) {
      return true;
  }
    return false;
  }
}
