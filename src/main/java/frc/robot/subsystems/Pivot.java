// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.utils.TalonPIDConfig;

public class Pivot extends SubsystemBase {
  public final WPI_TalonFX pivotMotor = new WPI_TalonFX(PivotConstants.PIVOT_MOTOR_CHANNEL);
  public final DigitalInput limitSwitch = new DigitalInput(PivotConstants.LIMIT_SWITCH_CHANNEL);
  public final TalonPIDConfig talonPIDConfig = PivotConstants.PIVOT_PID_CONFIG;

  /** Creates a new Pivot. */
  public Pivot() {
    talonPIDConfig.initializeTalonPID(pivotMotor, FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void tiltForward() {
    pivotMotor.set(ControlMode.PercentOutput, PivotConstants.PIVOT_SPEED);
  }

  public void tiltBackward() {
    pivotMotor.set(ControlMode.PercentOutput, -1*PivotConstants.PIVOT_SPEED);
  }

  public void stopPivotMotion() {
    pivotMotor.set(ControlMode.PercentOutput, 0);
  }

    public void pivotPID(double setpoint) {
    // Normalise setpoint
    setpoint = MathUtil.clamp(setpoint, talonPIDConfig.getLowerLimit(), talonPIDConfig.getUpperLimit());

    // Move arm toward setpoint
    pivotMotor.set(ControlMode.MotionMagic, setpoint);
  }


}
