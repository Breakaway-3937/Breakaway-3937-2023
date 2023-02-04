
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.AutoChooser;
import frc.robot.autos.AutoTrajectories;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

/* All variables, objects, and methods declared in lowerCamelCase */

public class RobotContainer {
  /* Controllers */
  private final Joystick translationController = new Joystick(0);
  private final Joystick rotationController = new Joystick(1);
  public final XboxController xboxController = new XboxController(2);
  public final Joystick buttonGrid = new Joystick(3);

  /* Drive Controls */
  private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
  private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
  private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;
  private final boolean openLoop = Constants.OPEN_LOOP;
  private final boolean fieldRelative = Constants.FIELD_RELATIVE;

  /* Driver Buttons */
  private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
  private final JoystickButton lbButton =  new JoystickButton(xboxController, Constants.Controllers.XBOXCONTROLLER_LB_BUTTON);
  private final JoystickButton rbButton =  new JoystickButton(xboxController, Constants.Controllers.XBOXCONTROLLER_RB_BUTTON);
  private final JoystickButton highLeft = new JoystickButton(buttonGrid, 0);
  private final JoystickButton highMid = new JoystickButton(buttonGrid, 0);
  private final JoystickButton highRight = new JoystickButton(buttonGrid, 0);
  private final JoystickButton midLeft = new JoystickButton(buttonGrid, 0); //FIXME
  private final JoystickButton midMid = new JoystickButton(buttonGrid, 0);
  private final JoystickButton midRight = new JoystickButton(buttonGrid, 0);
  private final JoystickButton lowLeft = new JoystickButton(buttonGrid, 0);
  private final JoystickButton lowMid = new JoystickButton(buttonGrid, 0);
  private final JoystickButton lowRight = new JoystickButton(buttonGrid, 0);


  /* Subsystems */
  public final DriveTrain s_DriveTrain = new DriveTrain();
  public final LED s_LED = new LED();
  public final PhotonVision s_Photon = new PhotonVision();
  public final Intake s_Intake = new Intake();
  public final Climber s_Climber = new Climber();
  public final Arm s_Arm = new Arm();
  
  /* Commands */


  /* Autos */
  private final AutoChooser autoChooser = new AutoChooser(new AutoTrajectories(Constants.DriveTrain.TRAJECTORY_CONSTRAINTS));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_DriveTrain.setDefaultCommand(new TeleopSwerve(s_DriveTrain, translationController, rotationController, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    Shuffleboard.getTab("Auto").add("Chooser", autoChooser.getModeChooser());
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
    translationButton.onTrue(new InstantCommand(() -> s_DriveTrain.zeroGyro()));
    highLeft.onTrue(new InstantCommand(() -> s_Photon.setHighLeft()));
    highRight.onTrue(new InstantCommand(() -> s_Photon.setHighRight()));
    highMid.onTrue(new InstantCommand(() -> s_Photon.setHighMid()));
    midLeft.onTrue(new InstantCommand(() -> s_Photon.setMidLeft()));
    midRight.onTrue(new InstantCommand(() -> s_Photon.setMidRight()));
    midMid.onTrue(new InstantCommand(() -> s_Photon.setMidMid()));
    lowMid.onTrue(new InstantCommand(() -> s_Photon.setHybridMid()));
    lowRight.onTrue(new InstantCommand(() -> s_Photon.setHybridRight()));
    lowLeft.onTrue(new InstantCommand(() -> s_Photon.setHybridLeft()));
    rbButton.onTrue(new InstantCommand(() -> s_LED.cone()));
    lbButton.onTrue(new InstantCommand(() -> s_LED.cube()));
  } 

  public DriveTrain getDrivetrain(){
    return s_DriveTrain;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command that is selected will run in autonomous
    return autoChooser.getCommand(this);
  }
}
