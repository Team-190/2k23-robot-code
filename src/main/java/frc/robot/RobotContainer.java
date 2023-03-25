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
import frc.robot.Constants.DrivetrainConstants.DRIVE_INPUT;
import frc.robot.Constants.DrivetrainConstants.DRIVE_STYLE;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.moveOut;
import frc.robot.commands.intake.ejectObject;
import frc.robot.commands.intake.intakeCone;
import frc.robot.commands.intake.intakeCube;
import frc.robot.commands.intake.intakeObject;
import frc.robot.commands.score;
import frc.robot.commands.stop;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.input.AttackThree;
import frc.robot.utils.input.XboxOneController;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TelescopingArm;
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

    public final IntakeSubsystem claw = new IntakeSubsystem();
    public final PivotSubsystem pivot = new PivotSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();

    //public boolean gamePiece = true; // true = cone & false = cube
    public int gamePiece = 0; // 0 = cube; 1 = cone; 2 = stow
   /** public enum GoalHeights {
        LOW(0),
        MID(1),
        HIGH(2);

        public final int targetGoal;
        private GoalHeights(int targetGoal) {
            this.targetGoal = targetGoal;
        }
    }

    public GoalHeights goalHeights; */
    public int goalHeight = 0; // 0 = Low; 1 = Mid; 2 = High; 3 = Single Player Pickup
    
    

    public final TelescopingArm telescopingArm = new TelescopingArm(0, 0, 0);

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


        operatorXboxController.yButton.onTrue(new SequentialCommandGroup(
            new InstantCommand(()-> setGoalHeight(2)),
            new moveOut(this))
        );
        operatorXboxController.bButton.onTrue(new SequentialCommandGroup(
            new InstantCommand(()-> setGoalHeight(1)),
            new moveOut(this))
        );
        operatorXboxController.aButton.onTrue(new SequentialCommandGroup(
            new InstantCommand(()-> setGoalHeight(0)),
            new moveOut(this))
        );
        operatorXboxController.leftBumper.onTrue(new InstantCommand(()-> setGamePiece(1))); // cone
        operatorXboxController.leftBumper.onTrue(new InstantCommand(()-> setGamePiece(0))); // cube
        
      //  leftStick.triggerButton.onTrue(new intakeCone(this));
        //trigger2.onTrue(new intakeCone(this));
        //faceButton.onTrue(new score(this));
      ///  rightStick.triggerButton.onTrue(new intakeCube(this));
        //rightButton.toggleOnTrue(new lift(this));
       // leftStick.rightFaceButton.onTrue(new stop(this));
       // leftStick.leftFaceButton.onTrue(new score(this));
        //leftbutton2.onTrue(new stop(this));
        //faceButton2.onTrue(new score(this));
       // leftStick.middleFaceButton.toggleOnTrue(new intakeObject(this));
       // rightStick.middleFaceButton.toggleOnTrue(new ejectObject(this));
        driveStyleChooser.addOption("Tank", DRIVE_STYLE.TANK);
        driveStyleChooser.addOption("Arcade", DRIVE_STYLE.ARCADE);
        driveStyleChooser.addOption("Curvature", DRIVE_STYLE.MCFLY);
        SmartDashboard.putData("DriveStyleChooser", driveStyleChooser);
        driveInputChooser.addOption("Joysticks", DRIVE_INPUT.JOYSTICKS);
        driveInputChooser.addOption("Controller", DRIVE_INPUT.CONTROLLER);
        SmartDashboard.putData("DriveInputChooser", driveInputChooser);
        SmartDashboard.putData("AutoModeChooser", autoModeChooser);
        SmartDashboard.putBoolean("Square Inputs?", true);

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
        //return autoModeChooser.getSelected();
        return new RunCommand(()-> this.drivetrainSubsystem.westCoastDrive(.5, .5, false), drivetrainSubsystem).withTimeout(1);
    }

    public void setDefaultCommands() {
        // Default drive command
       
        // turretSubsystem.setDefaultCommand(new VisionCommand(this));

        // turretSubsystem.setDefaultCommand(new VisionCommand(this));
         drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.arcadeDrive(driverXboxController.getLeftStickY(), 
         driverXboxController.getRightStickX(), false)));
         //drivetrainSubsystem.setDefaultCommand(new AutoBalance(drivetrainSubsystem));

        //drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.westCoastDrive(-leftStick.getY(), -rightStick.getY(), true), drivetrainSubsystem));
        //climberSubsystem.setDefaultCommand(new ClimberJumpGrabCommand(this));

    }

    public void setGamePiece(int value) {
        gamePiece = value;
    }

    public void setGoalHeight(int value) {
        goalHeight = value;
    }

    public void periodic() {}

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
