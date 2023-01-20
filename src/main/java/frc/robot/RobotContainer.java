
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  //private final Joystick translationController = new Joystick(0);
  //private final Joystick rotationController = new Joystick(1);
  public final XboxController xboxController = new XboxController(2);

  /* Drive Controls */
  /*private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
  private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
  private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;
  private final boolean openLoop = Constants.OPEN_LOOP;
  private final boolean fieldRelative = Constants.FIELD_RELATIVE;*/

  /* Driver Buttons */
  //private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);


  /* Subsystems */
  //public final DriveTrain s_DriveTrain = new DriveTrain();
  //private final CANdleSystem s_Candle = new CANdleSystem(xboxController);
  private final PhotonCameraWrapper s_PCW = new PhotonCameraWrapper();
  
  /* Commands */

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //s_DriveTrain.setDefaultCommand(new TeleopSwerve(s_DriveTrain, translationController, rotationController, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
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
    //translationButton.whenPressed(new InstantCommand(() -> s_DriveTrain.zeroGyro()));
    //translationButton.onTrue(new InstantCommand(() -> s_DriveTrain.zeroGyro()));
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command that is selected will run in autonomous
    return null;
  }
}
