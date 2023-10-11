// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Music extends CommandBase {
  private Drivetrain s_Drivetrain;
  private Orchestra orchestra;
  /** Creates a new Music. */
  public Music(Drivetrain s_Drivetrain) {
    this.s_Drivetrain = s_Drivetrain;
    orchestra = new Orchestra();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(int i = 0; i < 4; i++){
      orchestra.addInstrument(s_Drivetrain.swerveMods[i].getDriveMotor());
      orchestra.addInstrument(s_Drivetrain.swerveMods[i].getAngleMotor());
    }
    orchestra.loadMusic("output.chrp");
    orchestra.play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("RUNNING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ENDING");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
