// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.InputConstants;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.RobotContainer;
import frc.robot.utils.ArmUtils;
import frc.robot.utils.ArmUtils.GAME_PIECE;
import frc.robot.utils.ArmUtils.PIVOT_DIRECTION;

public class CANdleSubsystem extends SubsystemBase {
  public final CANdle candle = new CANdle(Constants.LEDConstants.LED_PORT, InputConstants.CANIVORE_BUS_NAME);
  public static int numLights = Constants.LEDConstants.LED_COUNT;
  RobotContainer robotContainer;
  Robot robot;
  ArmUtils armUtils;
  int r;
  int g;
  int b;
  int startIndex;

  /** Creates a new CANdleSubsystem. */
  public CANdleSubsystem(RobotContainer robotContainer, Robot robot) {
    this.robotContainer = robotContainer;
    this.robot = robot;
    armUtils = robotContainer.armUtils;
  }

  public void setTeleopLights() {
    if(armUtils.getGamePiece() == GAME_PIECE.CONE) {
      r = Constants.LEDConstants.YELLOW_R;
      g = Constants.LEDConstants.YELLOW_G;
      b = Constants.LEDConstants.YELLOW_B;
    } else {
      r = Constants.LEDConstants.PURPLE_R;
      g = Constants.LEDConstants.PURPLE_G;
      b = Constants.LEDConstants.PURPLE_B;
    }
    if(armUtils.getPivotDirection() == PIVOT_DIRECTION.FORWARD) {
      candle.setLEDs(r, g, b, 0, 0, numLights/4);
      candle.setLEDs(r, g, b, 0, 3*numLights/4, numLights/4);
    } else {
      candle.setLEDs(r, g, b, 0, numLights/2, numLights/2);
    }
    
    
  }

  public void setAutoLights() {

  }

  public void setIdleLights() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (robot.getRobotState() == RobotState.IDLE) {
      setIdleLights();
    } else if (robot.getRobotState() == RobotState.AUTO) {
      setAutoLights();
    } else if (robot.getRobotState() == RobotState.TELEOP) {
      setTeleopLights();
    }
  }
}
