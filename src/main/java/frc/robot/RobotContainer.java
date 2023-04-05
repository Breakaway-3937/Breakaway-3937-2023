
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.AutoChooser;
import frc.robot.autos.AutoTrajectories;
import frc.robot.commands.Align;
import frc.robot.commands.RunArm;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PhotonVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

/* All variables, objects, and methods declared in lowerCamelCase */

public class RobotContainer {
  /* Controllers */
  private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
  private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
  public final XboxController xboxController = new XboxController(Constants.Controllers.XBOX_CONTROLLER.getPort());
  public final Joystick buttonGrid = new Joystick(Constants.Controllers.BUTTON_GRID.getPort());

  /* Drive Controls */
  private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
  private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
  private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;
  private final boolean openLoop = Constants.OPEN_LOOP;
  private final boolean fieldRelative = Constants.FIELD_RELATIVE;


  /* Driver Buttons */
  private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
  private final JoystickButton rotationButton = new JoystickButton(rotationController, Constants.Controllers.ROTATION_BUTTON);
  private final JoystickButton highLeft = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_HIGH_LEFT);
  private final JoystickButton highMid = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_HIGH_MID);
  private final JoystickButton highRight = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_HIGH_RIGHT);
  private final JoystickButton midLeft = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_MID_LEFT);
  private final JoystickButton midMid = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_MID_MID);
  private final JoystickButton midRight = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_MID_RIGHT);
  private final JoystickButton hybridLeft = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_HYBRID_LEFT);
  private final JoystickButton hybridMid = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_HYBRID_MID);
  private final JoystickButton hybridRight = new JoystickButton(buttonGrid, Constants.Controllers.BUTTON_GRID_HYBRID_RIGHT);
  private final POVButton up = new POVButton(xboxController, Constants.Controllers.UP);
  private final POVButton right = new POVButton(xboxController, Constants.Controllers.RIGHT);
  private final POVButton down = new POVButton(xboxController, Constants.Controllers.DOWN);
  private final POVButton left = new POVButton(xboxController, Constants.Controllers.LEFT);

  /* Subsystems */
  public final Drivetrain s_Drivetrain = new Drivetrain();
  public final Intake s_Intake = new Intake();
  public final LED s_LED = new LED(s_Intake);
  public final PhotonVision s_Photon = new PhotonVision(s_LED, s_Intake);
  public final Climber s_Climber = new Climber();
  public final Arm s_Arm = new Arm();
  
  /* Commands */
  public final TeleopSwerve c_TeleopSwerve = new TeleopSwerve(s_Drivetrain, translationController, rotationController, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop);
  public final RunIntake c_RunIntake = new RunIntake(s_Intake, xboxController);
  public final RunClimber c_RunClimber = new RunClimber(s_Climber, xboxController);
  public final RunArm c_RunArm = new RunArm(s_Arm, xboxController, s_Photon);
  public final Align c_Align = new Align(s_Drivetrain);
  
  /* Autos */
  private final AutoTrajectories autoTrajectories = new AutoTrajectories();
  private final AutoChooser autoChooser = new AutoChooser(autoTrajectories, s_Drivetrain, s_Arm, s_Intake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(s_Drivetrain);
    CommandScheduler.getInstance().registerSubsystem(s_Intake);
    CommandScheduler.getInstance().registerSubsystem(s_LED);
    CommandScheduler.getInstance().registerSubsystem(s_Photon);
    CommandScheduler.getInstance().registerSubsystem(s_Climber);
    CommandScheduler.getInstance().registerSubsystem(s_Arm);

    s_Drivetrain.setDefaultCommand(c_TeleopSwerve);
    s_Intake.setDefaultCommand(c_RunIntake);
    s_Climber.setDefaultCommand(c_RunClimber);
    Shuffleboard.getTab("Auto").add("Chooser", autoChooser.getModeChooser());
    s_LED.cone();
    s_Intake.setCone();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    translationButton.onTrue(new InstantCommand(() -> s_Drivetrain.zeroGyro()));
    rotationButton.whileTrue(c_Align);
    highLeft.onTrue(new InstantCommand(() -> s_Photon.setHighLeft()));
    highRight.onTrue(new InstantCommand(() -> s_Photon.setHighRight()));
    highMid.onTrue(new InstantCommand(() -> s_Photon.setHighMid()));
    midLeft.onTrue(new InstantCommand(() -> s_Photon.setMidLeft()));
    midRight.onTrue(new InstantCommand(() -> s_Photon.setMidRight()));
    midMid.onTrue(new InstantCommand(() -> s_Photon.setMidMid()));
    hybridMid.onTrue(new InstantCommand(() -> s_Photon.setHybridMid()));
    hybridRight.onTrue(new InstantCommand(() -> s_Photon.setHybridRight()));
    hybridLeft.onTrue(new InstantCommand(() -> s_Photon.setHybridLeft()));
    up.onTrue(new InstantCommand(() -> s_LED.cone()).alongWith(new InstantCommand(() -> s_Intake.setCone())));
    right.onTrue(new InstantCommand(() -> s_LED.cube()).alongWith(new InstantCommand(() -> s_Intake.setCube())));
    down.onTrue(new InstantCommand(() -> s_Intake.setManualOverride(true)));
    left.onTrue(new InstantCommand(() -> s_Photon.setAuto()));
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command that is selected will run in autonomous
    return autoChooser.getCommand();
  }
}
