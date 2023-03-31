// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX clawMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CHANNEL);
  public final DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_CHANNEL);

  /** Creates a new Claw. */
  public IntakeSubsystem() {
    //clawMotor.configPeakCurrentLimit(35);
    //clawMotor.configContinuousCurrentLimit(35);
    // clawMotor.enableCurrentLimit(false);
    // SmartDashboard.putNumber("Current", clawMotor.getStatorCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // SmartDashboard.putNumber("Current", clawMotor.getStatorCurrent());

    
  }

  public void intake(){
    clawMotor.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED);
  }

  public void hold() {
    clawMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_SPEED_PASSIVE);
  }

  public void stop(){
    clawMotor.set(ControlMode.PercentOutput, 0);
  }

  public void score(){
    clawMotor.set(ControlMode.PercentOutput, -1*IntakeConstants.INTAKE_SPEED);
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

}
