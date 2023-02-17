// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RunTurret extends CommandBase {
  private final Arm s_Arm;
  private final Joystick joystick;
  private final XboxController xboxController;
  /** Creates a new RunTurret. */
  public RunTurret(Arm s_Arm, Joystick joystick, XboxController xboxController) {
    this.s_Arm = s_Arm;
    this.joystick = joystick;
    this.xboxController = xboxController;
    addRequirements(s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Arm.setRotation(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getRawButton(1)){
      s_Arm.setRotation(0.5);
    }
    else if (joystick.getRawButton(2)){
      s_Arm.setRotation(-0.5);
    }
    else if (joystick.getRawButton(6)){
      s_Arm.setRotation(0);
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
