// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.claw.EjectObject;
import frc.robot.utils.ArmUtils;
import frc.robot.utils.ArmUtils.ARM_STATE;
import frc.robot.utils.ArmUtils.GAME_PIECE;
import frc.robot.utils.ArmUtils.PIVOT_DIRECTION;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlacePieceHigh extends SequentialCommandGroup {
  /** Creates a new PlaceConeHigh. */
  ArmUtils armUtils;
  RobotContainer robotContainer;
  public PlacePieceHigh(RobotContainer conatiner, GAME_PIECE piece) {
    robotContainer = conatiner;
    armUtils = robotContainer.armUtils;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(armUtils.getMotionCommand(ARM_STATE.HIGH, piece, PIVOT_DIRECTION.REVERSE).withTimeout(3), 
    (new EjectObject(robotContainer)).withTimeout(1),
    armUtils.getMotionCommand(ARM_STATE.LOW, piece, PIVOT_DIRECTION.FORWARD).withTimeout(3)

    );
  }
}
