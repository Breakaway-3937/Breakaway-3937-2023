// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TestTurret extends CommandBase {
  private final Arm s_Arm;
  private final XboxController xboxController;
  /** Creates a new TestTurret. */
  public TestTurret(Arm s_Arm, XboxController xboxController) {
    this.s_Arm = s_Arm;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Arm.setTurret(0);
    s_Arm.setWrist(0);
    s_Arm.setExtension(-20);
    s_Arm.setShoulder(-10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getRawButton(1)){
      s_Arm.setWrist(15);
    }
    else if(xboxController.getRawButton(2)){
      s_Arm.setWrist(0);
    }
    else if(xboxController.getRawButton(3)){
      s_Arm.setWrist(30);
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
