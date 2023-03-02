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
  //private final PhotonVision s_Photon;
  private final XboxController xboxController;
  private double shoulderPosition, turretPosition, extensionPosition, wristPosition;
  private int state;
  private boolean flag, flag1;
  private Joystick joystick;
  /** Creates a new RunArm. */
  //public RunArm(Arm s_Arm, Joystick joystick, PhotonVision s_Photon, XboxController xboxController){
  public RunArm(Arm s_Arm, Joystick joystick, XboxController xboxController){
    this.s_Arm = s_Arm;
    //this.s_Photon = s_Photon;
    this.xboxController = xboxController;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //s_Photon.setAllFalse();
    shoulderPosition = -10;
    extensionPosition = -20;
    wristPosition = 0;
    turretPosition = 0;
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
    /*if(s_Photon.getSelectedScore().get(0) || s_Photon.getSelectedScore().get(1) || s_Photon.getSelectedScore().get(2)){
      shoulderPosition = -13;
      extensionPosition = -46000;
      wristPosition = 36;
      turretPosition = 0;
      state = 0;
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
    }*/
    if(joystick.getRawButton(1) || joystick.getRawButton(2) || joystick.getRawButton(3)){
      shoulderPosition = -13;
      extensionPosition = -46000;
      wristPosition = 36;
      turretPosition = 0;
      state = 0;
    }
    else if(joystick.getRawButton(4) || joystick.getRawButton(5) || joystick.getRawButton(6)){
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
    }
    else if(joystick.getRawButton(7) || joystick.getRawButton(8) || joystick.getRawButton(9)){
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
      //s_Photon.setAllFalse();
      if(Intake.getDeadCone()){
        shoulderPosition = 0;
        if(extensionPosition > -13000){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -13000;
        wristPosition = 6.5;
        turretPosition = 0;
        flag = false;
      }
      else if(Intake.getConeCubeMode() && !Intake.getDeadCone()){
        shoulderPosition = -0.5;
        if(extensionPosition > -20){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -20;
        wristPosition = 12.45;
        turretPosition = 0;
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
      }   
    }
    //Jack Arm
    else if(xboxController.getRawButton(4)){
      //s_Photon.setAllFalse();
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
        flag = false;
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
      }
    }
    else if(xboxController.getRawButton(3)){
      //s_Photon.setAllFalse();
      shoulderPosition = -10;
      if(extensionPosition == -20 || extensionPosition == -13000){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -20;
      wristPosition = 0;
      turretPosition = 0;
    }
    else if(xboxController.getRawButton(2)){
      //s_Photon.setAllFalse();
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
    }
    else if(xboxController.getRawButton(5)){
      //s_Photon.setAllFalse();
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
    }
    if(RunClimber.dropArm){
      shoulderPosition = -0.5;
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

    if((shoulderPosition == -0.5 || shoulderPosition == 0) && s_Arm.getShoulder1Position() < shoulderPosition + 1 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
      s_Arm.stopShoulder();
      flag1 = true;
    }
    else{
      flag1 = false;
    }

    switch(state){
      case 0: 
        if(!flag1){
          s_Arm.setShoulder(shoulderPosition);
        }
        if(s_Arm.getShoulder1Position() < shoulderPosition + 0.5 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
          //s_Arm.setRotation(turretPosition);
          flag = true;
        }
        if(s_Arm.getRotationPosition() < turretPosition + 0.5 && s_Arm.getRotationPosition() > turretPosition - 0.5 && flag){
          s_Arm.setExtension(extensionPosition);
          s_Arm.setWrist(wristPosition);
          flag = false;
        }
        break;
      case 1: 
        s_Arm.setWrist(wristPosition);
        s_Arm.setExtension(extensionPosition);
        if(s_Arm.getExtensionPosition() < extensionPosition + 125 && s_Arm.getExtensionPosition() > extensionPosition - 125){
          //s_Arm.setRotation(turretPosition);
          flag = true;
        }
        if(s_Arm.getRotationPosition() < turretPosition + 0.5 && s_Arm.getRotationPosition() > turretPosition - 0.5 && flag && !flag1){
          s_Arm.setShoulder(shoulderPosition);
          flag = false;
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
