// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetIntake extends CommandBase {
  private final XboxController xboxController;
  private final Arm s_Arm;
  /** Creates a new RunWrist. */
  public SetIntake(XboxController xboxController , Arm s_Arm) {
    this.xboxController = xboxController;
    this.s_Arm = s_Arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getRawButton(1)){
      s_Arm.setShoulder(-5);
      s_Arm.setExtension(20);
    }
    else if(xboxController.getRawButton(2)){
      s_Arm.setShoulder(-5);
      s_Arm.setExtension(10000);
    }
    else if(xboxController.getRawButton(3)){
      s_Arm.setShoulder(-5);
      s_Arm.setExtension(14000);
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
