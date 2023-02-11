// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunWrist extends CommandBase {
  private final Intake s_Intake;
  private final XboxController xboxController;
  /** Creates a new RunWrist. */
  public RunWrist(Intake s_Intake, XboxController xboxController) {
    this.xboxController = xboxController;
    this.s_Intake = s_Intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getRawButton(8)){
      s_Intake.setWrist(5);
      System.out.println("wrist");
    }
    else if(xboxController.getRawButton(7)){
      s_Intake.setWrist(15);
    }
    else if(xboxController.getRawButton(11)){
      s_Intake.setWrist(15);
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
