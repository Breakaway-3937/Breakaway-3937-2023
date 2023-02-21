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
  private double shoulderPosition, turretPosition, extensionPosition, wristPosition;
  private int state;
  /** Creates a new RunArm. */
  public RunArm(Arm s_Arm, Joystick joystick, PhotonVision s_Photon, XboxController xboxController){
    this.joystick = joystick;
    this.s_Arm = s_Arm;
    this.s_Photon = s_Photon;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm, s_Photon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Arm.setShoulder(0);
    s_Arm.setExtension(-10);
    s_Arm.setWrist(0);
    s_Arm.setRotation(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if(joystick.getRawButton(0)){
      if(s_Photon.getSelectedScore().get(0)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(1)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(2)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(3)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(4)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(5)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(6)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(7)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(8)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
      if(s_Arm.getShoulder1Position() < armPosition + 0.2 && s_Arm.getShoulder1Position() > armPosition - 0.2){
        s_Arm.setExtension(-s_Arm.getScoreLength());
      }
    }*/
    if(xboxController.getPOV() == 0){
      shoulderPosition = -12;
      extensionPosition = -56000;
      wristPosition = 39.5;
      turretPosition = 0;
      state = 0;
    }
    else if(xboxController.getPOV() == 90){
      shoulderPosition = -13;
      if(extensionPosition > -27775){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -27775;
      wristPosition = 51.7;
      turretPosition = 0;
    }
    else if(xboxController.getPOV() == 180){
      shoulderPosition = -5;
      if(extensionPosition > -307){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -307;
      wristPosition = 35;
    }
    if(xboxController.getRawButton(1)){
      if(Intake.getConeCubeMode()){
        shoulderPosition = -0.2;
        if(extensionPosition > -3526){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -3526;
        wristPosition = 10;
        turretPosition = 0;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -0.21;
        if(extensionPosition > -13108){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -13108;
        wristPosition = 7.7;
        turretPosition = 0;
      }    
    }
    else if(xboxController.getRawButton(4)){
      shoulderPosition = -15;
      if(extensionPosition > -14845){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -14845;
      wristPosition = 53;
      turretPosition = 0;
    }
    else if(xboxController.getRawButton(3)){
      shoulderPosition = -15;
      if(extensionPosition > -10){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = 10;
      wristPosition = 0;
      turretPosition = 0;
      state = 1;
    }
    else if(xboxController.getRawButton(2)){
      shoulderPosition = -7.1;
      if(extensionPosition > -417){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -417;
      wristPosition = 0.2;
      turretPosition = 0;
    }
    else if(xboxController.getRawButton(5)){
      shoulderPosition = -7;
      if(extensionPosition > -50){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -50;
      turretPosition = 0;
      wristPosition = 0;
    }

    switch(state){
      case 0: 
        s_Arm.setShoulder(shoulderPosition);
        if(s_Arm.getShoulder1Position() < shoulderPosition + 0.2 && s_Arm.getShoulder1Position() > shoulderPosition - 0.2){
          s_Arm.setRotation(turretPosition);
        }
        else if(s_Arm.getRotationPosition() < turretPosition + 0.2 && s_Arm.getRotationPosition() > turretPosition - 0.2){
          s_Arm.setExtension(extensionPosition);
        }
        else if(s_Arm.getExtensionPosition() < extensionPosition + 0.2 && s_Arm.getExtensionPosition() > extensionPosition - 0.2){
          s_Arm.setWrist(wristPosition);
        }
        break;
      case 1: 
        s_Arm.setWrist(wristPosition);
        if(s_Arm.getWrist() < wristPosition + 0.2 && s_Arm.getWrist() > wristPosition - 0.2){
          s_Arm.setExtension(extensionPosition);
        }
        else if(s_Arm.getExtensionPosition() < extensionPosition + 0.2 && s_Arm.getExtensionPosition() > extensionPosition - 0.2){
          s_Arm.setRotation(turretPosition);
        }
        else if(s_Arm.getRotationPosition() < turretPosition + 0.2 && s_Arm.getRotationPosition() > turretPosition - 0.2){
          s_Arm.setShoulder(shoulderPosition);
        }
        break;
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
