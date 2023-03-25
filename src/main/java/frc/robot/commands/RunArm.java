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
  private double shoulderPosition, turretPosition, wristPosition;
  public static double extensionPosition;
  private int state;
  private boolean flag, flag1, track, ready;
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
    extensionPosition = -100;
    wristPosition = 0;
    turretPosition = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Photon.getSelectedScore().get(0) || s_Photon.getSelectedScore().get(1) || s_Photon.getSelectedScore().get(2)){
      shoulderPosition = -12.1;
      extensionPosition = -46500;
      wristPosition = 36;
      turretPosition = 0;
      state = 0;
      track = true;
      ready = false;
    }
    else if(s_Photon.getSelectedScore().get(3) || s_Photon.getSelectedScore().get(4) || s_Photon.getSelectedScore().get(5)){
      shoulderPosition = -11.85;
      if(!flag1 && extensionPosition > -26628){
        state = 0;
      }
      else{
        state = 1;
        flag1 = true;
      }
      extensionPosition = -26628;
      wristPosition = 50;
      turretPosition = 0;
      track = true;
      ready = false;
    }
    else if(s_Photon.getSelectedScore().get(6) || s_Photon.getSelectedScore().get(7) || s_Photon.getSelectedScore().get(8)){
      shoulderPosition = -5;
      if(!flag1 && extensionPosition > -307){
        state = 0;
      }
      else{
        state = 1;
        flag1 = true;
      }
      extensionPosition = -307;
      wristPosition = 35;
      turretPosition = 0;
      track = false;
      ready = false;
    }
    //Ground Intake
    if(xboxController.getRawButton(1)){
      s_Photon.setAllFalse();
      //Dead Cone
      if(Intake.getDeadCone()){
        shoulderPosition = 0;
        if(!flag1 && extensionPosition > -15256){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -15256;
        wristPosition = 12;
        turretPosition = 0;
        track = false;
      }
      //Cone
      else if(Intake.getConeCubeMode()){
        shoulderPosition = 0;
        if(!flag1 && extensionPosition > -192){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -192;
        wristPosition = 14;
        turretPosition = 0;
        track = false;
      }
      //Cube
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -0.5;
        if(!flag1 && extensionPosition > -759){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -759;
        wristPosition = 25;
        turretPosition = 0;
        track = false;
      }   
    }
    //Jack Arm
    else if(xboxController.getRawButton(4)){
      s_Photon.setAllFalse();
      shoulderPosition = -13;
      if(!flag1 && extensionPosition > -28000){
        state = 0;
      }
      else{
        state = 1;
        flag1 = true;
      }
      extensionPosition = -28000;
      wristPosition = 46;
      turretPosition = 0;
      track = false;
    }
    else if(xboxController.getRawButton(3)){
      s_Photon.setAllFalse();
      shoulderPosition = -10;
      state = 1;
      flag1 = true;
      extensionPosition = -100;
      wristPosition = 0;
      turretPosition = 0;
      track = false;
    }
    else if(xboxController.getRawButton(2)){
      s_Photon.setAllFalse();
      shoulderPosition = -5.5;
      if(!flag1 && extensionPosition > -417){
        state = 0;
      }
      else{
        state = 1;
        flag1 = true;
      }
      extensionPosition = -417;
      wristPosition = 0.2;
      turretPosition = 0;
      track = false;
    }
    if(xboxController.getRawButton(5)){
      shoulderPosition = -0.5;
        if(!flag1 && extensionPosition > -100){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -100;
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
        flag1 = false;
      }
      if(s_Arm.getShoulder1Position() < shoulderPosition + 0.5 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
        if(s_Photon.getAuto() && track && ready){
          turretPosition = s_Photon.getAutoTrackAngle();
          if(turretPosition >= -5 && turretPosition <= 5){
            s_Arm.setTurret(turretPosition);
            s_Arm.setExtension(extensionPosition);
            s_Arm.setWrist(wristPosition);
          }
        }
        else if(!s_Photon.getAuto() || !track){
          if(turretPosition >= -5 && turretPosition <= 5){
            s_Arm.setTurret(turretPosition);
            s_Arm.setExtension(extensionPosition);
            s_Arm.setWrist(wristPosition);
          }
        }
      }
      break;

      case 1:
      if(s_Photon.getAuto() && track && ready){
        turretPosition = s_Photon.getAutoTrackAngle();
        if(turretPosition >= -5 && turretPosition <= 5){
          s_Arm.setTurret(turretPosition);
          s_Arm.setExtension(extensionPosition);
          s_Arm.setWrist(wristPosition);
        }
      }
      else if(!s_Photon.getAuto() || !track){
        if(turretPosition >= -5 && turretPosition <= 5){
          s_Arm.setTurret(turretPosition);
          s_Arm.setExtension(extensionPosition);
          s_Arm.setWrist(wristPosition);
        }
      }
      if(s_Arm.getExtensionPosition() < extensionPosition + 125 && s_Arm.getExtensionPosition() > extensionPosition - 125 && !flag){
        s_Arm.setShoulder(shoulderPosition);
        flag1 = false;
      }
      break;
      
    }
  }

  public static double getExtensionValue(){
    return extensionPosition;
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
