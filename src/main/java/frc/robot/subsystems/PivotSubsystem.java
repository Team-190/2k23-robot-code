// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.utils.TalonPIDConfig;

public class PivotSubsystem extends SubsystemBase {
  public final WPI_TalonFX pivotMotor = new WPI_TalonFX(PivotConstants.PIVOT_MOTOR_CHANNEL);
  public final DigitalInput limitSwitch = new DigitalInput(PivotConstants.LIMIT_SWITCH_CHANNEL);
  public final TalonPIDConfig talonPIDConfig = PivotConstants.PIVOT_PID_CONFIG;

  /** Creates a new Pivot. */
  public PivotSubsystem() {
    talonPIDConfig.initializeTalonPID(pivotMotor, FeedbackDevice.IntegratedSensor);
    pivotMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean pivotup = SmartDashboard.getBoolean("Pivot Positive", false);
    boolean pivotdown = SmartDashboard.getBoolean("Pivot Negative", false);
    boolean pivotstop = SmartDashboard.getBoolean("Pivot Stop", false);
    if (pivotup) {
      tiltForward();
    }
    if (pivotdown) {
      tiltBackward();
    }
    if (pivotstop) {
      stopPivotMotion();
      SmartDashboard.putBoolean("Pivot Positive", false);
      SmartDashboard.putBoolean("Pivot Negative", false);
    }
    SmartDashboard.putNumber("Pivot Position", pivotMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Pivot Motion Complete", isMotionCompleted());
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }
  public void setPIDDefault() {
    pivotMotor.configMotionCruiseVelocity(talonPIDConfig.rpmToTicksPer100ms(PivotConstants.PIVOT_MOTOR_VELOCITY));
  }
  public void setPIDStow() {
    pivotMotor.configMotionCruiseVelocity(talonPIDConfig.rpmToTicksPer100ms(PivotConstants.PIVOT_MOTOR_VELOCITY_STOW));
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

  public boolean isMotionCompleted() {
    double error = pivotMotor.getClosedLoopTarget() - pivotMotor.getSelectedSensorPosition();
    return Math.abs(error) < Constants.PivotConstants.CODE_TOLERANCE;
  }

  public void pivotPID(double setpoint) {
    // Normalise setpoint
    setpoint = MathUtil.clamp(setpoint, talonPIDConfig.getLowerLimit(), talonPIDConfig.getUpperLimit());

    // Move arm toward setpoint
    pivotMotor.set(ControlMode.MotionMagic, setpoint);
  }

  public void pivotRelativePID(double setpoint) {
    double target = pivotMotor.getClosedLoopTarget() + setpoint;

    target = MathUtil.clamp(target, talonPIDConfig.getLowerLimit(), talonPIDConfig.getUpperLimit());

    // Move arm toward setpoint
    pivotMotor.set(ControlMode.MotionMagic, target);
  }

  public double ticksToDegrees(double ticks) {
    return ticks/Constants.PivotConstants.PIVOT_TICKS_PER_DEGREE;
  }

  public double degreesToTicks(double degrees) {
    return degrees*Constants.PivotConstants.PIVOT_TICKS_PER_DEGREE;
  }


}
