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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConstants.DRIVE_INPUT;
import frc.robot.Constants.DrivetrainConstants.DRIVE_STYLE;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.intakeCone;
import frc.robot.commands.lift;
import frc.robot.commands.score;
import frc.robot.commands.stop;
import frc.robot.commands.intakeCube;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DrivetrainSubsystem;

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

    /*
    * Subsystems
    */
    public final DrivetrainSubsystem drivetrainSubsystem =
            new DrivetrainSubsystem(
                    Constants.DrivetrainConstants.P,
                    Constants.DrivetrainConstants.I,
                    Constants.DrivetrainConstants.D);

    public final Claw claw = new Claw();

    public final Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    public boolean compressorEnabled = compressor.isEnabled();
    public boolean compressorPressureSwitch = compressor.getPressureSwitchValue();
    public double compressorCurrent = compressor.getCurrent();

    /*
    * Input
    */
   public final Joystick leftStick =
           new Joystick(Constants.InputConstants.LEFT_JOYSTICK_CHANNEL);
    public final Joystick rightStick =
            new Joystick(Constants.InputConstants.RIGHT_JOYSTICK_CHANNEL);
    public final XboxController driverXboxController =
            new XboxController(Constants.InputConstants.XBOX_CHANNEL);

    public final JoystickButton trigger = new JoystickButton(leftStick, 1);
    public final JoystickButton faceButton = new JoystickButton(leftStick, 2);
    public final JoystickButton middleButton = new JoystickButton(leftStick, 3);
    public final JoystickButton rightButton = new JoystickButton(leftStick, 4);
    public final JoystickButton leftButton = new JoystickButton(leftStick, 5);
    public final JoystickButton trigger2 = new JoystickButton(rightStick,  1);
    public final JoystickButton leftbutton2 = new JoystickButton(rightStick, 5);
    public final JoystickButton faceButton2 = new JoystickButton(rightStick, 2);

    //  public final ButtonBoxLeft buttonBoxLeft = new ButtonBoxLeft(2);
    // public final ButtonBoxRight buttonBoxRight = new ButtonBoxRight(3);


    /**
    * Constructor for the robot container Called when the Rio is powered on, and is only called once.
    * We use this to configure commands from buttons and default commands
    */
    PathPlannerTrajectory autoPath = PathPlanner.loadPath("New Path", new PathConstraints(1, 1));
    public RobotContainer() {

        /*
        for (int i = 1500; i < 6001; i += 50) {
            shooterRPMChooser.addOption(""+i+ " RPM", i);
        }
        */

        trigger.onTrue(new intakeCube(this));
        trigger2.onTrue(new intakeCone(this));
        faceButton.onTrue(new score(this));
        rightButton.toggleOnTrue(new lift(this));
        leftButton.onTrue(new stop(this));
        leftbutton2.onTrue(new stop(this));
        faceButton2.onTrue(new score(this));

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
        return null;
    }

    public void setDefaultCommands() {
        // Default drive command
       
        // turretSubsystem.setDefaultCommand(new VisionCommand(this));

        // turretSubsystem.setDefaultCommand(new VisionCommand(this));
         drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(this));

        //drivetrainSubsystem.setDefaultCommand(new RunCommand(()-> drivetrainSubsystem.westCoastDrive(-leftStick.getY(), -rightStick.getY(), true), drivetrainSubsystem));
        //climberSubsystem.setDefaultCommand(new ClimberJumpGrabCommand(this));

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
