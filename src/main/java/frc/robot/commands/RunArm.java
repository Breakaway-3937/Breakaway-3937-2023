//FIXME
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class RunArm extends CommandBase {
  private final Arm s_Arm;
  private final Joystick joystick;
  private final PhotonVision s_Photon;
  private final XboxController xboxController;
  private double armPosition;
  /** Creates a new RunArm. */
  public RunArm(Arm s_Arm, Joystick joystick, PhotonVision s_Photon, XboxController xboxController) {
    this.joystick = joystick;
    this.s_Arm = s_Arm;
    this.s_Photon = s_Photon;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm, s_Photon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getRawButton(0)){
      if(s_Photon.getSelectedScore().get(0)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      else if(s_Photon.getSelectedScore().get(1)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      else if(s_Photon.getSelectedScore().get(2)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      else if(s_Photon.getSelectedScore().get(3)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      else if(s_Photon.getSelectedScore().get(4)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      else if(s_Photon.getSelectedScore().get(5)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      else if(s_Photon.getSelectedScore().get(6)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      else if(s_Photon.getSelectedScore().get(7)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      else if(s_Photon.getSelectedScore().get(8)){
        s_Arm.setShoulder(0);
        armPosition = 0;
      }
      if(s_Arm.getShoulder1Position() < armPosition + 0.2 && s_Arm.getShoulder1Position() > armPosition - 0.2){
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
    }
    else if(xboxController.getRawButton(1)){
      s_Arm.setRotation(0);
    }
    else if(xboxController.getRawButton(4)){
      s_Arm.setRotation(0);
    }
    else if(xboxController.getRawButton(3)){
      s_Arm.setExtension(10);
      if(s_Arm.getExtensionPosition() < 10 + 300 && s_Arm.getExtensionPosition() > -100){
        s_Arm.setShoulder(1);
      }
      if(s_Arm.getShoulder1Position() < 1 + 0.2 && s_Arm.getShoulder1Position() > 1 - 0.2){
        s_Arm.setRotation(0);
      }
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
