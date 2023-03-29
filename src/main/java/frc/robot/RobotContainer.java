/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.DRIVE_INPUT;
import frc.robot.Constants.DrivetrainConstants.DRIVE_STYLE;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoBalanceSequence;
import frc.robot.commands.auto.ScoreMidDriveBack;
import frc.robot.commands.pivot.MoveToPivotPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.ArmUtils;
import frc.robot.utils.ArmUtils.ARM_STATE;
import frc.robot.utils.ArmUtils.GAME_PIECE;
import frc.robot.utils.ArmUtils.PIVOT_DIRECTION;
import frc.robot.utils.input.AttackThree;
import frc.robot.utils.input.XboxOneController;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

    // Choosers
    // public final SendableChooser<Integer> shooterRPMChooser = new SendableChooser<>();
    public final SendableChooser<Command> autoModeChooser = new SendableChooser<>();
    public final SendableChooser<DRIVE_STYLE> driveStyleChooser = new SendableChooser<>();
    public final SendableChooser<DRIVE_INPUT> driveInputChooser = new SendableChooser<>();
    // public final SendableChooser<Integer> topRPMChooser = new SendableChooser<>();

    // Cameras
    UsbCamera camera1;
    public LimeLightSubsystem limeLightSubsystem;
    /*
    * Subsystems
    */
    public final DrivetrainSubsystem drivetrainSubsystem =
            new DrivetrainSubsystem(
                    Constants.DrivetrainConstants.P,
                    Constants.DrivetrainConstants.I,
                    Constants.DrivetrainConstants.D,
                    this);

    public final ElevatorSubsystem telescopingArm = new ElevatorSubsystem(0, 0, 0);
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final PivotSubsystem pivot = new PivotSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();

    public final ArmUtils armUtils = new ArmUtils(this, GAME_PIECE.CUBE, PIVOT_DIRECTION.FORWARD, ARM_STATE.STOW);

    //public boolean gamePiece = true; // true = cone & false = cube


; 
//public int goalHeight = 0; // 0 = Low; 1 = Mid; 2 = High; 3 = Single Player Pickup; 4 = Double Player Pickup; 5 = Floor intake
    

    public final Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    public boolean compressorEnabled = compressor.isEnabled();
    public boolean compressorPressureSwitch = compressor.getPressureSwitchValue();
    public double compressorCurrent = compressor.getCurrent();

    /*
    * Input
    */
 public final AttackThree leftStick =
           new AttackThree(Constants.InputConstants.LEFT_JOYSTICK_CHANNEL);
    public final AttackThree rightStick =
            new AttackThree(Constants.InputConstants.RIGHT_JOYSTICK_CHANNEL);
    public final XboxOneController driverXboxController =
            new XboxOneController(Constants.InputConstants.XBOX_DR_CHANNEL);
    public final XboxOneController operatorXboxController = 
            new XboxOneController(Constants.InputConstants.XBOX_OP_CHANNEL);

    PathPlannerTrajectory autoPath = PathPlanner.loadPath("New Path", new PathConstraints(1, 1));


    /**
    * Constructor for the robot container Called when the Rio is powered on, and is only called once.
    * We use this to configure commands from buttons and default commands
    */
    public RobotContainer() {

        /*
        for (int i = 1500; i < 6001; i += 50) {
            shooterRPMChooser.addOption(""+i+ " RPM", i);
        }
        */


       operatorXboxController.leftBumper.onTrue(new InstantCommand(()-> armUtils.setGamePiece(GAME_PIECE.CONE))); // cone
       operatorXboxController.rightBumper.onTrue(new InstantCommand(()-> armUtils.setGamePiece(GAME_PIECE.CUBE))); // cube
       operatorXboxController.startButton.onTrue(new InstantCommand(()-> armUtils.setPivotDirection(PIVOT_DIRECTION.REVERSE))); // cone
       operatorXboxController.selectButton.onTrue(new InstantCommand(()-> armUtils.setPivotDirection(PIVOT_DIRECTION.FORWARD))); // cube
       // operatorXboxController.yButton.onTrue(new RunCommand(()-> pivot.pivotPID(-126500)));
 /**      operatorXboxController.yButton.onTrue(new MoveToPivotPosition(this, -126500));
       //operatorXboxController.xButton.onTrue(new MoveToPivotPosition(this, 0));
       operatorXboxController.xButton.onTrue(new SequentialCommandGroup(
        new MoveToPivotPosition(this, 0),
        new MoveToArmPosition(this, 0),
        new MoveToWristPosition(this, 0)
        ));
       operatorXboxController.aButton.onTrue(new MoveToPivotPosition(this, -225000));
     /**  operatorXboxController.bButton.onTrue(new SequentialCommandGroup(
        new MoveToPivotPosition(this, Constants.PivotConstants.CUBE_HIGH_GOAL_PIVOT_TICKS),
        new MoveToArmPosition(this, Constants.ArmConstants.CUBE_HIGH_GOAL_EXT_TICKS)
        )
    );
    operatorXboxController.bButton.onTrue(new SequentialCommandGroup(
        new MoveToPivotPosition(this, Constants.PivotConstants.CONE_MID_GOAL_PIVOT_TICKS),
        new MoveToArmPosition(this, Constants.ArmConstants.CONE_MID_GOAL_EXT_TICKS),
        new MoveToWristPosition(this, Constants.WristConstants.FORWARD_RIGHT_ANGLE)
        )
    );
    operatorXboxController.bButton.onTrue(new SequentialCommandGroup(
        new MoveToPivotPosition(this, Constants.PivotConstants.CONE_HIGH_GOAL_PIVOT_TICKS),
        new MoveToArmPosition(this, Constants.ArmConstants.CONE_HIGH_GOAL_EXT_TICKS),
        new MoveToWristPosition(this, Constants.WristConstants.FORWARD_RIGHT_ANGLE)
        )
    );*/
       // operatorXboxController.xButton.onTrue(new RunCommand(()-> pivot.pivotPID(0)));
       // operatorXboxController.aButton.onTrue(new RunCommand(()-> pivot.pivotPID(-225000)));
        
      /**  driverXboxController.yButton.onTrue(new InstantCommand(() -> drivetrainSubsystem.setBreakMode()));
        new Trigger(()-> driverXboxController.getRightTrigger() > 0.5).whileTrue(new RunCommand(()-> intake.intake()){}).onFalse(new InstantCommand(()-> intake.clawMotor.set(ControlMode.PercentOutput, .1)));
        new Trigger(()-> driverXboxController.getLeftTrigger() > 0.5).whileTrue(new RunCommand(()-> intake.score()){}).onFalse(new InstantCommand(()-> intake.stop()));
*/
        leftStick.triggerButton.whileTrue(new RunCommand(()-> intake.intake()){}).onFalse(new InstantCommand(()-> intake.clawMotor.set(ControlMode.PercentOutput, .1)));
        leftStick.middleFaceButton.onTrue(new InstantCommand(()-> drivetrainSubsystem.setCoastMode()));

        rightStick.triggerButton.whileTrue(new RunCommand(()-> intake.score()){}).onFalse(new InstantCommand(()-> intake.clawMotor.set(ControlMode.PercentOutput, 0)));
        rightStick.middleFaceButton.onTrue(new InstantCommand(()-> drivetrainSubsystem.setBreakMode()));
        

        
        driveStyleChooser.addOption("Tank", DRIVE_STYLE.TANK);
        driveStyleChooser.addOption("Arcade", DRIVE_STYLE.ARCADE);
        driveStyleChooser.addOption("Curvature", DRIVE_STYLE.MCFLY);
        SmartDashboard.putData("DriveStyleChooser", driveStyleChooser);
        driveInputChooser.addOption("Joysticks", DRIVE_INPUT.JOYSTICKS);
        driveInputChooser.addOption("Controller", DRIVE_INPUT.CONTROLLER);
        SmartDashboard.putData("DriveInputChooser", driveInputChooser);
        SmartDashboard.putBoolean("Square Inputs?", true);

        autoModeChooser.addOption("ScoreMidDriveBack", new ScoreMidDriveBack(this));
        autoModeChooser.addOption("DriveForward", new RunCommand(()-> new RunCommand(()-> this.drivetrainSubsystem.westCoastDrive(.25, .25, false), drivetrainSubsystem).withTimeout(2)));
        autoModeChooser.addOption("DoNothing", new InstantCommand());
        autoModeChooser.setDefaultOption("DriveForward", new RunCommand(()-> new RunCommand(()-> this.drivetrainSubsystem.westCoastDrive(.25, .25, false), drivetrainSubsystem).withTimeout(2)));
        SmartDashboard.putData("AutoModeChooser", autoModeChooser);
        // SmartDashboard.putData("Set Flywheel RPM", shooterRPMChooser);

        // initializeCamera();

        // Left Stick Bindings

         // Left Stick Bindings

         
        // leftStick.triggerButton.whenPressed(new ToggleCollectorCommandGroup(this, 0.75));
        
        // Until interupt / timeout may fix stuff
        
        setDefaultCommands();

        //drivetrainSubsystem.gyro.calibrate();


        

        
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // return null;
        //return armUtils.getMotionCommand(ARM_STATE.MID, GAME_PIECE.CUBE, ROBOT_SIDE.FORWARD);
        return autoModeChooser.getSelected();
        //return new RunCommand(()-> this.drivetrainSubsystem.westCoastDrive(.25, .25, false), drivetrainSubsystem).withTimeout(2);
        //return new autoBalanceSequence(this);
        //return new ScoreMidDriveBack(this);
    }
   // SlewRateLimiter leftLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT);
   // SlewRateLimiter rightLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT);

    public void setDefaultCommands() {
        // Default drive command
       
        // turretSubsystem.setDefaultCommand(new VisionCommand(this));

        // turretSubsystem.setDefaultCommand(new VisionCommand(this));
     //    drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.arcadeDrive(driverXboxController.getLeftStickY()/Math.pow(2,0.5), 
      //   -1*driverXboxController.getRightStickX()/Math.pow(2,0.5), true), drivetrainSubsystem));
         //drivetrainSubsystem.setDefaultCommand(new AutoBalance(drivetrainSubsystem));


        drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.arcadeDrive(-leftStick.getY(), -rightStick.getX(), true), drivetrainSubsystem));
       // drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.westCoastDrive(-leftStick.getY(), -rightStick.getY(), true), drivetrainSubsystem));
        // drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.westCoastDrive(-leftStick.getY(), -rightStick.getY(), false), drivetrainSubsystem));
        //climberSubsystem.setDefaultCommand(new ClimberJumpGrabCommand(this));

    }

    public void periodic() {
        SmartDashboard.putNumber("Wrist Setpoint", armUtils.wristSetpoint());
        SmartDashboard.putNumber("Arm Setpoint", armUtils.armSetpoint());
        SmartDashboard.putNumber("Pivot Setpoint", armUtils.pivotSetpoint());
    }

    /**
    * Initializes the camera(s)
    */
   public void initializeCamera() {
    try {
    } catch (VideoException e) {
      e.printStackTrace();
    }
   }
}
