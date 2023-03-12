// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.TalonPIDConfig;

/**
 * This class containes the telescoping arm, pivot, and wrist
 * 
 * If someone changes speeds or anything like that -- DO IT IN CONSTANTS, NOT HARDCODED INTO THIS FILE
 */
public class TelescopingArm extends PIDSubsystem {
  public final WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.ARM_MOTOR_CHANNEL);
  public final DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_CHANNEL);
  public final TalonPIDConfig talonPIDConfig = ArmConstants.ARM_PID_CONFIG;
  
  /** Creates a new TelescopingArm. */
  public TelescopingArm(double P, double I, double D) {
    super(new PIDController(P, I, D));
    talonPIDConfig.initializeTalonPID(armMotor, FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

   /**
   * Converts rotations per minute to ticks per 100ms
   * Multiplies rotations per minute by ticks per rotation
   * Divides by 600 to convert from minute to millisecond
   * @param rpm input of rotations per minute
   * @return output of ticks per 100ms
   */
  public double rpmToTicksPer100ms(double rpm) {
    return rpm * ArmConstants.ARM_TICKS_PER_ROTATION / 600;
  }

   /**
   * Convert from inches to ticks
   * @param degrees current degree amount
   * @return current encoder tick value
   */
  public double inchesToTicks (double inches){
    return inches * ArmConstants.TICKS_PER_INCH;
  }

  public void configPIDF(WPI_TalonFX motorController, double P, double I, double D, double F) {
    motorController.config_kP(ArmConstants.SLOT_ID, P);
    motorController.config_kI(ArmConstants.SLOT_ID, I);
    motorController.config_kD(ArmConstants.SLOT_ID, D);
    motorController.config_kF(ArmConstants.SLOT_ID, F);
}

@Override
public void useOutput(double output, double setpoint) {
  // Use the output here
}

@Override
public double getMeasurement() {
  // Return the process variable measurement here
  return armMotor.getSelectedSensorPosition();
}

 /**
   * Move arm to setpoint
   * @param setpoint encoder tick value for turret to move to
   */
  public void armPID(double setpoint) {
    // Normalise setpoint
    setpoint = MathUtil.clamp(setpoint, talonPIDConfig.getLowerLimit(), talonPIDConfig.getUpperLimit());

    // Move arm toward setpoint
    armMotor.set(ControlMode.MotionMagic, setpoint);
  }

   /**
   * Move arm relatively by setpoint 
   * @param setpoint encoder tick value to move turret by
   */
  public void relativeArmPID(double setpoint) {
    armPID(armMotor.getClosedLoopTarget() + setpoint);
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


}
