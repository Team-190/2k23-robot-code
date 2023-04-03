// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeObject extends CommandBase {

  IntakeSubsystem clawsubsystem;
  boolean intakeStarted;
  /** Creates a new intakeCube. */
  public IntakeObject(RobotContainer robotcontainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    clawsubsystem = robotcontainer.intake;
    addRequirements(clawsubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawsubsystem.intake();
    intakeStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (clawsubsystem.clawMotor.getSelectedSensorVelocity() > IntakeConstants.MOTION_TOLERANCE) intakeStarted = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawsubsystem.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    return (intakeStarted && Math.abs(clawsubsystem.clawMotor.getSelectedSensorVelocity()) < IntakeConstants.MOTION_TOLERANCE);
  }
}
