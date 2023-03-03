// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoBalanceAuto extends CommandBase {
  private final DriveTrain s_Drivetrain;
  private double value = 0;
  private double acceptable = 2;
  /** Creates a new AutoBalanceAuto. */
  public AutoBalanceAuto(DriveTrain s_Drivetrain) {
    this.s_Drivetrain = s_Drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    value = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Drivetrain.drive(new Translation2d(value, 0), 0, true, false);
    if((s_Drivetrain.getYaw().getDegrees() % 360 > 315 || s_Drivetrain.getYaw().getDegrees() % 360 < 45) && Math.abs(s_Drivetrain.getRoll()) > acceptable){
      if(s_Drivetrain.getRoll() < 0){
        value = (Math.abs(s_Drivetrain.getRoll()) - acceptable) * -0.015;
      }
      else{
        value = (Math.abs(s_Drivetrain.getRoll()) - acceptable) * 0.015;
      }
    }
    else if((s_Drivetrain.getYaw().getDegrees() % 360 > 135 || s_Drivetrain.getYaw().getDegrees() % 360 < 225) && Math.abs(s_Drivetrain.getRoll()) > acceptable){
      if(s_Drivetrain.getRoll() > 0){
        value = (Math.abs(s_Drivetrain.getRoll()) - acceptable) * -0.015;
      }
      else{
        value = (Math.abs(s_Drivetrain.getRoll()) - acceptable) * 0.015;
      }
    }
    else if(Math.abs(s_Drivetrain.getRoll()) < acceptable){
      s_Drivetrain.drive(new Translation2d(-value, 0), 0.3, true, false);
      s_Drivetrain.drive(new Translation2d(0, 0), 0, true, false);
    }
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
