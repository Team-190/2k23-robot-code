// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.utils.TalonPIDConfig;

public class WristSubsystem extends SubsystemBase {
  public final WPI_TalonFX wristMotor = new WPI_TalonFX(WristConstants.WRIST_MOTOR_CHANNEL);
  public final TalonPIDConfig talonPIDConfig = WristConstants.WRIST_PID_CONFIG;

  /** Creates a new Wrist. */
  public WristSubsystem() {
    talonPIDConfig.initializeTalonPID(wristMotor, FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void wristDown() {
    wristMotor.set(ControlMode.PercentOutput, -1*WristConstants.WRIST_SPEED);
  }

  public void wristUp() {
    wristMotor.set(ControlMode.PercentOutput, WristConstants.WRIST_SPEED);
  }

  public void stopWristMotion() {
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isMotionCompleted() {
    return Math.abs(wristMotor.getClosedLoopError()) < Constants.WristConstants.TOLERANCE;
  }


  public void wristPID(double setpoint) {
    // Normalise setpoint
    setpoint = MathUtil.clamp(setpoint, talonPIDConfig.getLowerLimit(), talonPIDConfig.getUpperLimit());

    // Move arm toward setpoint
    wristMotor.set(ControlMode.MotionMagic, setpoint);
  }

}
