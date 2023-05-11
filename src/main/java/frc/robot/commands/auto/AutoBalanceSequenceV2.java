// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.claw.EjectObject;
import frc.robot.commands.claw.IntakeObject;
import frc.robot.utils.ArmUtils.ARM_STATE;
import frc.robot.utils.ArmUtils.GAME_PIECE;
import frc.robot.utils.ArmUtils.PIVOT_DIRECTION;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceSequenceV2 extends SequentialCommandGroup {
  /** Creates a new autoBalanceSequence. */

  public AutoBalanceSequenceV2(RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeObject(robotContainer), 
      //new ParallelDeadlineGroup(new WaitCommand(2), robotContainer.armUtils.getMotionCommand(ARM_STATE.HIGH, GAME_PIECE.CUBE, PIVOT_DIRECTION.FORWARD)), 
      robotContainer.armUtils.getMotionCommand(ARM_STATE.HIGH, GAME_PIECE.CUBE, PIVOT_DIRECTION.REVERSE),
      new EjectObject(robotContainer).withTimeout(0.1),
      new ParallelDeadlineGroup(
      new DriveOverChargeStation(robotContainer, false),
      robotContainer.armUtils.getMotionCommand(ARM_STATE.STATION_SINGLE, GAME_PIECE.CUBE, PIVOT_DIRECTION.FORWARD)),
      new WaitCommand(1),
      new AutoBalanceV2(robotContainer.drivetrainSubsystem, true)
    );
  }
}
