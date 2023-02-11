// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class SetIntake extends CommandBase {
  private final Intake s_Intake;
  private final XboxController xboxController;
  private final Arm s_Arm;
  /** Creates a new RunWrist. */
  public SetIntake(Intake s_Intake, XboxController xboxController , Arm s_Arm) {
    this.xboxController = xboxController;
    this.s_Intake = s_Intake;
    this.s_Arm = s_Arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake, s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getRawButton(1)){
      s_Arm.setShoulder(0);
      s_Arm.setExtension(0);
      s_Intake.setWrist(5);
    }
    else if(xboxController.getRawButton(4)){
      s_Arm.setShoulder(0);
      s_Arm.setExtension(0);
      s_Intake.setWrist(15);
    }
    else if(xboxController.getRawButton(11)){
      s_Arm.setShoulder(0);
      s_Arm.setExtension(50);
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
