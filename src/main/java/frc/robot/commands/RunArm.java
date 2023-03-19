// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class RunArm extends CommandBase {
  private final Arm s_Arm;
  private final PhotonVision s_Photon;
  private final XboxController xboxController;
  private double shoulderPosition, turretPosition, extensionPosition, wristPosition;
  private int state;
  private boolean flag, track, ready;
  /** Creates a new RunArm. */
  public RunArm(Arm s_Arm, XboxController xboxController, PhotonVision s_Photon){
    this.s_Arm = s_Arm;
    this.s_Photon = s_Photon;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Photon.setAllFalse();
    shoulderPosition = -10;
    extensionPosition = -20;
    wristPosition = 0;
    turretPosition = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Photon.getSelectedScore().get(0) || s_Photon.getSelectedScore().get(1) || s_Photon.getSelectedScore().get(2)){
      shoulderPosition = -13;
      extensionPosition = -46500;
      wristPosition = 36;
      turretPosition = 0;
      state = 0;
      track = true;
      ready = false;
    }
    else if(s_Photon.getSelectedScore().get(3) || s_Photon.getSelectedScore().get(4) || s_Photon.getSelectedScore().get(5)){
      shoulderPosition = -12.75;
      if(extensionPosition > -24500){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -24500;
      wristPosition = 43.8;
      turretPosition = 0;
      track = true;
      ready = false;
    }
    else if(s_Photon.getSelectedScore().get(6) || s_Photon.getSelectedScore().get(7) || s_Photon.getSelectedScore().get(8)){
      shoulderPosition = -5;
      if(extensionPosition > -307){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -307;
      wristPosition = 35;
      track = false;
      ready = false;
    }
    if(xboxController.getRawButton(1)){
      s_Photon.setAllFalse();
      if(Intake.getDeadCone()){
        shoulderPosition = 0;
        if(extensionPosition > -9000){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -9000;
        wristPosition = 8;
        turretPosition = 0;
        track = false;
      }
      else if(Intake.getConeCubeMode()){
        shoulderPosition = -0.5;
        if(extensionPosition > -20){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -20;
        wristPosition = 12;
        turretPosition = 0;
        track = false;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -0.5;
        if(extensionPosition > -20){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -20;
        wristPosition = 19.85;
        turretPosition = 0;
        track = false;
      }   
    }
    //Jack Arm
    else if(xboxController.getRawButton(4)){
      s_Photon.setAllFalse();
      if(Intake.getConeCubeMode()){
        shoulderPosition = -14;
        if(extensionPosition > -32000){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -32000;
        wristPosition = 47;
        turretPosition = 0;
        track = false;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -13.26;
        if(extensionPosition > -30200){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -30200;
        wristPosition = 44.76;
        turretPosition = 0;
        track = false;
      }
    }
    else if(xboxController.getRawButton(3)){
      s_Photon.setAllFalse();
      shoulderPosition = -10;
      if(extensionPosition == -20 || extensionPosition == -9000){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -20;
      wristPosition = 0;
      turretPosition = 0;
      track = false;
    }
    else if(xboxController.getRawButton(2)){
      s_Photon.setAllFalse();
      shoulderPosition = -6.5;
      if(extensionPosition > -417){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -417;
      wristPosition = 0.2;
      turretPosition = 0;
      track = false;
    }
    else if(xboxController.getRawButton(5)){
      s_Photon.setAllFalse();
      shoulderPosition = -10;
      if(extensionPosition > -20){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -20;
      turretPosition = 0;
      wristPosition = 0;
      track = false;
    }
    if(xboxController.getRawButton(5)){
      shoulderPosition = -0.5;
        if(extensionPosition > -20){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -20;
        wristPosition = 0;
        turretPosition = 0;
        track = false;
    }

    if((shoulderPosition == -0.5 || shoulderPosition == 0) && s_Arm.getShoulder1Position() < shoulderPosition + 1 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
      s_Arm.stopShoulder();
      flag = true;
    }
    else{
      flag = false;
    }

    if(xboxController.getRawButton(8)){
      ready = true;
    }

    switch(state){
      case 0:
      if(!flag){
        s_Arm.setShoulder(shoulderPosition);
      }
      if(s_Arm.getShoulder1Position() < shoulderPosition + 0.5 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
        if(s_Photon.getAuto() && track && ready){
          turretPosition = s_Photon.getAutoTrackAngle();
          extensionPosition = s_Photon.getAutoTrackDistance();
          if(extensionPosition >= -46500 && extensionPosition <= -20 && turretPosition >= -5 && turretPosition <= 5){
            s_Arm.setTurret(turretPosition);
            s_Arm.setExtension(extensionPosition);
            s_Arm.setWrist(wristPosition);
          }
        }
        else{
          s_Arm.setTurret(turretPosition);
          s_Arm.setExtension(extensionPosition);
          s_Arm.setWrist(wristPosition);
        }
      }
      break;

      case 1:
      if(s_Photon.getAuto() && track && ready){
        turretPosition = s_Photon.getAutoTrackAngle();
        extensionPosition = s_Photon.getAutoTrackDistance();
        if(extensionPosition >= -46500 && extensionPosition <= -20 && turretPosition >= -5 && turretPosition <= 5){
          s_Arm.setTurret(turretPosition);
          s_Arm.setExtension(extensionPosition);
          s_Arm.setWrist(wristPosition);
        }
      }
      else{
        s_Arm.setTurret(turretPosition);
        s_Arm.setExtension(extensionPosition);
        s_Arm.setWrist(wristPosition);
      }
      if(s_Arm.getExtensionPosition() < extensionPosition + 125 && s_Arm.getExtensionPosition() > extensionPosition - 125 && !flag){
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
