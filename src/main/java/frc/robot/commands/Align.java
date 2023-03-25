// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Align extends CommandBase {
  private final Drivetrain s_Drivetrain;
  private double xAxis;
  private double yAxis;
  private double rAxis;
  private double rotation;
  private Translation2d translation;
  /** Creates a new Align. */
  public Align(Drivetrain s_Drivetrain) {
    this.s_Drivetrain = s_Drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yAxis = Constants.Controllers.TRANSLATION_CONTROLLER.getRawAxis(Constants.Controllers.STRAFE_AXIS);
    xAxis = Constants.Controllers.TRANSLATION_CONTROLLER.getRawAxis(Constants.Controllers.TRANSLATION_AXIS);
    rAxis = Constants.Controllers.ROTATION_CONTROLLER.getRawAxis(Constants.Controllers.ROTATION_AXIS);
    
    /* Deadbands */
    yAxis = (Math.abs(yAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : xAxis;
    

    if((DriverStation.getAlliance().toString().equals("Blue") && s_Drivetrain.getPose().getX() > 4 && s_Drivetrain.getPose().getX() < 9.5) || (DriverStation.getAlliance().toString().equals("Red") && 16.5 - s_Drivetrain.getPose().getX() > 4 && 16.5 - s_Drivetrain.getPose().getX() < 9.5)){
      rAxis = (Math.abs(rAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : rAxis;
      rotation = rAxis * Constants.Drivetrain.MAX_ANGULAR_VELOCITY;
    }
    else if((DriverStation.getAlliance().toString().equals("Blue") && s_Drivetrain.getPose().getX() < 4) || (DriverStation.getAlliance().toString().equals("Red") && 16.5 - s_Drivetrain.getPose().getX() < 4)){
      if(s_Drivetrain.getYaw().getDegrees() % 360 > 180){
        rAxis = 0.8;
      }
      else if(s_Drivetrain.getYaw().getDegrees() % 360 < 180){
        rAxis = -0.8;
      }
      else{
        rAxis = 0;
      }
      rotation = rAxis * Constants.Drivetrain.MAX_ANGULAR_VELOCITY;
    }
    else if((DriverStation.getAlliance().toString().equals("Blue") && s_Drivetrain.getPose().getX() > 9.5) || (DriverStation.getAlliance().toString().equals("Red") && 16.5 - s_Drivetrain.getPose().getX() > 9.5)){
        if(s_Drivetrain.getYaw().getDegrees() % 360 > 90){
          rAxis = 0.8;
        }
        else if(s_Drivetrain.getYaw().getDegrees() % 360 < 90){
          rAxis = -0.8;
        }
        else{
          rAxis = 0;
        }
        rotation = rAxis * Constants.Drivetrain.MAX_ANGULAR_VELOCITY;
      }
    translation = new Translation2d(yAxis, xAxis).times(Constants.Drivetrain.MAX_SPEED);
    s_Drivetrain.drive(translation, rotation, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}