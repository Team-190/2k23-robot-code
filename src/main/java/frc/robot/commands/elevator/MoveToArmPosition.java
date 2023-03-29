// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class MoveToArmPosition extends CommandBase {
  ElevatorSubsystem elevator;
  int setpoint;
  RobotContainer robotcontainer;
  /** Creates a new moveToPivotPosition. */
  public MoveToArmPosition(RobotContainer robotcontainer, int setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotcontainer = robotcontainer;
    elevator = robotcontainer.telescopingArm;
    this.setpoint = setpoint;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.armPID(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pivot.pivotPID(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isMotionCompleted();
  }
}
