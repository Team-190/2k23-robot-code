// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;

/**
 * This class containes the telescoping arm, pivot, and wrist
 * 
 * If someone changes speeds or anything like that -- DO IT IN CONSTANTS, NOT HARDCODED INTO THIS FILE
 */
public class TelescopingArm extends SubsystemBase {
  
  public final WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.ARM_MOTOR_CHANNEL);
  public final WPI_TalonFX pivotMotor = new WPI_TalonFX(ArmConstants.PIVOT_MOTOR_CHANNEL);
  public final WPI_TalonFX wristMotor = new WPI_TalonFX(ArmConstants.WRIST_MOTOR_CHANNEL);
  
  /** Creates a new TelescopingArm. */
  public TelescopingArm() {
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    armMotor.setSelectedSensorPosition(0);
    pivotMotor.setSelectedSensorPosition(0);
    wristMotor.setSelectedSensorPosition(0);
  }

  public void scoringState() { // should be updated with more updated logic/numbers
    moveTo(armMotor, ArmConstants.ARM_SCORING_POSITION);
    moveTo(pivotMotor, ArmConstants.PIVOT_SCORING_POSITION);
    moveTo(wristMotor, ArmConstants.WRIST_SCORING_POSITION);
  }

  public void collectingState() { // should be updated with more updated logic/numbers
    moveTo(armMotor, ArmConstants.ARM_COLLECTING_POSITION);
    moveTo(pivotMotor, ArmConstants.PIVOT_COLLECTING_POSITION);
    moveTo(wristMotor, ArmConstants.WRIST_COLLECTING_POSITION);
  }

  // this should be replaced with PID later on
  public void moveTo(WPI_TalonFX motor, double inches) { // conversion between ticks and inches should be calculated and included in the code
    if (inches > motor.getSelectedSensorPosition()) {
      while (inches > motor.getSelectedSensorPosition()) {
        armMotor.set(ControlMode.PercentOutput, ArmConstants.ARM_SPEED);
      }
    } else if (inches < motor.getSelectedSensorPosition()) {
      while (inches < armMotor.getSelectedSensorPosition()) {
        armMotor.set(ControlMode.PercentOutput, -1*ArmConstants.ARM_SPEED);
      }
    }
  }

  public void extend() {
    armMotor.set(ControlMode.PercentOutput, ArmConstants.ARM_SPEED);
  }

  public void retract() {
    armMotor.set(ControlMode.PercentOutput, -1*ArmConstants.ARM_SPEED);
  }

  public void stopArmMotion () {
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  public void tiltForward() {
    pivotMotor.set(ControlMode.PercentOutput, ArmConstants.PIVOT_SPEED);
  }

  public void tiltBackward() {
    pivotMotor.set(ControlMode.PercentOutput, -1*ArmConstants.PIVOT_SPEED);
  }

  public void stopPivotMotion() {
    pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  public void wristDown() {
    wristMotor.set(ControlMode.PercentOutput, -1*ArmConstants.WRIST_SPEED);
  }

  public void wristUp() {
    wristMotor.set(ControlMode.PercentOutput, ArmConstants.WRIST_SPEED);
  }

  public void stopWristMotion() {
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stop() {
    stopArmMotion();
    stopPivotMotion();
    stopWristMotion();
  }
}
