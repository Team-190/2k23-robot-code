// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ClawConstants;

// public class Claw extends SubsystemBase {

//   public final CANSparkMax clawMotor = new CANSparkMax(ClawConstants.CLAW_MOTOR_CHANNEL, MotorType.kBrushless);
//   public final DoubleSolenoid coneCylinder = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.CLAW_SOLENOID_CHANNEL_1, ClawConstants.CLAW_SOLENOID_CHANNEL_2);
//   public final Solenoid liftCylinder = new Solenoid(PneumaticsModuleType.REVPH, ClawConstants.CLAW_LIFT_SOLENOID_CHANNEL);
//   /** Creates a new Claw. */
//   public Claw() {
//     //clawMotor.configPeakCurrentLimit(35);
//     //clawMotor.configContinuousCurrentLimit(35);
//     clawMotor.setSmartCurrentLimit(40, 80); 
//     // SmartDashboard.putNumber("Current", clawMotor.getStatorCurrent());
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
    
//     SmartDashboard.putNumber("Current", clawMotor.getOutputCurrent());
//     SmartDashboard.putNumber("Claw Velocity", clawMotor.getEncoder().getVelocity());

    
//   }

//   public void intake(){
//     clawMotor.set(-0.8);
//   }

//   public void stop(){
//     clawMotor.stopMotor();
//   }

//   public void score(){
//     clawMotor.set(0.3);
//   }

//   public void collectMode(Value state){
//     coneCylinder.set(state);
//   }

//   public void lift(boolean state){
//     liftCylinder.set(state);
//   }

// }
