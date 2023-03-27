// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multisubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.ChangeArmPosition;
import frc.robot.commands.pivot.ChangePivotPosition;
import frc.robot.commands.wrist.ChangeWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveOut extends SequentialCommandGroup {
  /** Creates a new moveToScore. */
  public MoveOut(RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ChangePivotPosition(robotContainer),
      new ChangeArmPosition(robotContainer),
      new ChangeWristPosition(robotContainer)
    );
  }
}
