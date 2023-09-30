/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants.DRIVE_INPUT;
import frc.robot.Constants.DrivetrainConstants.DRIVE_STYLE;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoBalanceSequence;
import frc.robot.commands.auto.AutoBalanceSequenceV2;
import frc.robot.commands.auto.AutoBalanceV2;
import frc.robot.commands.auto.DriveOverChargeStation;
import frc.robot.commands.auto.HighConeBalance;
import frc.robot.commands.auto.MidCubeBalance;
import frc.robot.commands.auto.PathPlannerFollowCommand;
import frc.robot.commands.auto.PlacePieceHigh;
import frc.robot.commands.auto.ScoreHighConeDriveBack;
import frc.robot.commands.auto.ScoreHighCubeDriveBack;
import frc.robot.commands.auto.ScoreMidDriveBack;
import frc.robot.commands.claw.EjectObject;
import frc.robot.commands.claw.IntakeObject;
import frc.robot.commands.drivetrain.DriveWithVision;
import frc.robot.commands.pivot.RelativePivotPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.ArmUtils;
import frc.robot.utils.ArmUtils.ARM_STATE;
import frc.robot.utils.ArmUtils.GAME_PIECE;
import frc.robot.utils.ArmUtils.PIVOT_DIRECTION;
import frc.robot.utils.input.AttackThree;
import frc.robot.utils.input.XboxOneController;

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
    public LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
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
    public final IntakeSubsystem intake = new IntakeSubsystem(this);
    public final PivotSubsystem pivot = new PivotSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();

    public boolean moveArmFinished = false;
    public final ArmUtils armUtils = new ArmUtils(this, GAME_PIECE.CUBE, PIVOT_DIRECTION.FORWARD, ARM_STATE.STOW);

    //public boolean gamePiece = true; // true = cone & false = cube


; 
//public int goalHeight = 0; // 0 = Low; 1 = Mid; 2 = High; 3 = Single Player Pickup; 4 = Double Player Pickup; 5 = Floor intake
    
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

    PathPlannerTrajectory autoPath = PathPlanner.loadPath("TrajectoryTest", new PathConstraints(1, 1));


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
        operatorXboxController.bButton.onTrue(armUtils.getMotionCommand(ARM_STATE.MID));
        operatorXboxController.yButton.onTrue(armUtils.getMotionCommand(ARM_STATE.HIGH));
        operatorXboxController.aButton.onTrue(armUtils.getMotionCommand(ARM_STATE.LOW));
        operatorXboxController.xButton.onTrue(armUtils.getMotionCommand(ARM_STATE.STOW));
        //new Trigger(operatorXboxController.povRight()).onTrue(armUtils.getMotionCommand(ARM_STATE.STATION_SINGLE));
        new POVButton(operatorXboxController, 270).onTrue(armUtils.getMotionCommand(ARM_STATE.STATION_SINGLE));
        new POVButton(operatorXboxController, 0).onTrue(new InstantCommand(()-> armUtils.setPivotDirection(PIVOT_DIRECTION.FORWARD)));
        new POVButton(operatorXboxController, 180).onTrue(new InstantCommand(()-> armUtils.setPivotDirection(PIVOT_DIRECTION.REVERSE)));

        new Trigger(()-> Math.abs(operatorXboxController.getLeftStickY()) > 0.2)
            .whileTrue(new RelativePivotPIDCommand(this, ()-> 1.5 * operatorXboxController.getLeftStickY()));

        new Trigger(()-> Math.abs(operatorXboxController.getRightTrigger()) > 0.25).whileTrue(new IntakeObject(this));
    
        
        
      /**  driverXboxController.yButton.onTrue(new InstantCommand(() -> drivetrainSubsystem.setBreakMode()));
        new Trigger(()-> driverXboxController.getRightTrigger() > 0.5).whileTrue(new RunCommand(()-> intake.intake()){}).onFalse(new InstantCommand(()-> intake.clawMotor.set(ControlMode.PercentOutput, .1)));
        new Trigger(()-> driverXboxController.getLeftTrigger() > 0.5).whileTrue(new RunCommand(()-> intake.score()){}).onFalse(new InstantCommand(()-> intake.stop()));
*/
      /**  rightStick.triggerButton.whileTrue(new IntakeObject(this)
            .andThen(armUtils.getMotionCommand(ARM_STATE.STOW)));
        rightStick.triggerButton.onTrue(new InstantCommand(()-> armUtils.setPivotDirection(PIVOT_DIRECTION.FORWARD))
            .andThen(armUtils.getMotionCommand(ARM_STATE.LOW)));
        leftStick.triggerButton.whileTrue(new EjectObject(this));*/

        leftStick.middleFaceButton.onTrue(new InstantCommand(()-> drivetrainSubsystem.setCoastMode()));
        rightStick.middleFaceButton.onTrue(new InstantCommand(()-> drivetrainSubsystem.setBreakMode()));
        rightStick.bottomFaceButton.whileTrue(new IntakeObject(this)
        .andThen(armUtils.getMotionCommand(ARM_STATE.STOW)));
        rightStick.bottomFaceButton.onTrue(new InstantCommand(()-> armUtils.setPivotDirection(PIVOT_DIRECTION.FORWARD))
        .andThen(armUtils.getMotionCommand(ARM_STATE.LOW)));
        leftStick.bottomFaceButton.whileTrue(new EjectObject(this));
        // rightStick.bottomFaceButton.whileTrue(new RunCommand(()-> drivetrainSubsystem.seekTarget()));
        
        // driveStyleChooser.addOption("Tank", DRIVE_STYLE.TANK);
        // driveStyleChooser.addOption("Arcade", DRIVE_STYLE.ARCADE);
        // driveStyleChooser.addOption("Curvature", DRIVE_STYLE.MCFLY);
        // SmartDashboard.putData("DriveStyleChooser", driveStyleChooser);
        // driveInputChooser.addOption("Joysticks", DRIVE_INPUT.JOYSTICKS);
        // driveInputChooser.addOption("Controller", DRIVE_INPUT.CONTROLLER);
        // SmartDashboard.putData("DriveInputChooser", driveInputChooser);
        // SmartDashboard.putBoolean("Square Inputs?", true);

        // autoModeChooser.addOption("ScoreMidDriveBackDriver", new ScoreMidDriveBack(this));
        // autoModeChooser.addOption("DriveForwardField", new RunCommand(()-> this.drivetrainSubsystem.westCoastDrive(.25, .25, false), drivetrainSubsystem).withTimeout(2));
        // autoModeChooser.addOption("DoNothing", new InstantCommand());
        // autoModeChooser.addOption("Turn Right", new PathPlannerFollowCommand(this, true, "TrajectoryTest"));
        // autoModeChooser.addOption("ConeThenCube", new PathPlannerFollowCommand(this, true, "ScoringTest Copy"));
        // autoModeChooser.addOption("ScorePickup1", new PathPlannerFollowCommand(this, true, "ScorePickup1"));
        // autoModeChooser.addOption("ScoreCubeHighAndBalanceField", new SequentialCommandGroup(new AutoBalanceSequence(this)));
        // autoModeChooser.addOption("BalanceField", new SequentialCommandGroup(new AutoBalanceV2(drivetrainSubsystem, false)));
        // autoModeChooser.addOption("ScoreCubeHighMobilityThenBalanceField", new SequentialCommandGroup(new AutoBalanceSequenceV2(this)));
        // autoModeChooser.addOption("ScoreHighCubeDriveBackDriver", new SequentialCommandGroup(new ScoreHighCubeDriveBack(this)));
        // autoModeChooser.addOption("ScoreHighConeDriveBackDriver", new SequentialCommandGroup(new ScoreHighConeDriveBack(this)));
        // autoModeChooser.addOption("ScoreHighConeBalanceField", new SequentialCommandGroup(new HighConeBalance(this)));


        SmartDashboard.putData("AutoModeChooser", autoModeChooser);
        

        SmartDashboard.putString("Game Piece", armUtils.getGamePiece().name());
        SmartDashboard.putString("Pivot Direction", armUtils.getPivotDirection().name());

        SmartDashboard.putBoolean("Wrist Positive", false);
        SmartDashboard.putBoolean("Wrist Negative", false);
        SmartDashboard.putBoolean("Pivot Positive", false);
        SmartDashboard.putBoolean("Pivot Negative", false);
        SmartDashboard.putBoolean("Pivot Stop", false);
        SmartDashboard.putBoolean("Wrist Stop", false);

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
        // return autoModeChooser.getSelected();
        return null;
        //return new PathPlannerFollowCommand(this, true, "TrajectoryTest");
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

        // drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.westCoastDrive(-leftStick.getY(), -rightStick.getY(), true), drivetrainSubsystem));
        // drivetrainSubsystem.setDefaultCommand(new DriveWithVision(this, ()-> -MathUtil.clamp(leftStick.getY(), -0.8, 0.8),  ()-> -MathUtil.clamp(rightStick.getY(), -0.8, 0.8), true, ()-> false/*rightStick.getBottomFaceButton()*/));
        // drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.arcadeDrive(0, 0, moveArmFinished);), null));
        drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.arcadeDrive(-leftStick.getY(), -rightStick.getX(), true), drivetrainSubsystem));
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
