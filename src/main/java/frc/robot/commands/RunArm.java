// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class RunArm extends CommandBase {
  private final Arm s_Arm;
  private final PhotonVision s_Photon;
  private final XboxController xboxController;
  private double turretPosition, wristPosition;
  public static double shoulderPosition, extensionPosition;
  private int state;
  private boolean flag, flag1, track, intake, manualOverride, wristFirst;
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
    wristPosition = 10;
    turretPosition = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Photon.getSelectedScore().get(0) || s_Photon.getSelectedScore().get(1) || s_Photon.getSelectedScore().get(2)){
      intake = false;
      wristFirst = false;
      if(Intake.getConeCubeMode()){
        shoulderPosition = -13;
        extensionPosition = -44500;
        wristPosition = 60;
        turretPosition = 0;
        state = 0;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -11.25;
        extensionPosition = -44500;
        wristPosition = 44.2;
        turretPosition = 0;
        state = 0;
      }
      track = true;
    }
    else if(s_Photon.getSelectedScore().get(3) || s_Photon.getSelectedScore().get(4) || s_Photon.getSelectedScore().get(5)){
      intake = false;
      wristFirst = false;
      if(Intake.getConeCubeMode()){
        shoulderPosition = -13.75;
        if(!flag1 && extensionPosition >= -19500){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -19500;
        wristPosition = 66.5;
        turretPosition = 0;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -10.75;
        if(!flag1 && extensionPosition >= -19000){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -19000;
        wristPosition = 44.2;
        turretPosition = 0;
      }
      track = true;
    }
    else if(s_Photon.getSelectedScore().get(6) || s_Photon.getSelectedScore().get(7) || s_Photon.getSelectedScore().get(8)){
      intake = false;
      wristFirst = false;
      shoulderPosition = 0;
      if(!flag1 && extensionPosition >= -100){
        state = 0;
      }
      else{
        state = 1;
        flag1 = true;
      }
      extensionPosition = -100;
      wristPosition = 13.5;
      turretPosition = 0;
      track = false;
    }
    if(xboxController.getPOV() == Constants.Controllers.DOWN){
      wristFirst = true;
      manualOverride = true;
      Robot.m_robotContainer.s_Intake.setManualOverride(true);
      intake = true;
      s_Photon.setAllFalse();
      shoulderPosition = 0;
      if(!flag1 && extensionPosition >= -100){
        state = 0;
      }
      else{
        state = 1;
        flag1 = true;
      }
      extensionPosition = -100;
      wristPosition = 2;
      turretPosition = 0;
      track = false; 
    }
    //Ground Intake
    else if(xboxController.getRawButton(1)){
      wristFirst = true;
      manualOverride = false;
      Robot.m_robotContainer.s_Intake.setManualOverride(false);
      intake = true;
      s_Photon.setAllFalse();
      if(Intake.getConeCubeMode()){
        shoulderPosition = 0;
        if(!flag1 && extensionPosition >= -100){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -100;
        wristPosition = 29;
        turretPosition = 0;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = 0;
        if(!flag1 && extensionPosition >= -100){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -100;
        wristPosition = 30;
        turretPosition = 0;
      }
      track = false; 
    }
    //Jack Arm
    else if(xboxController.getRawButton(4)){
      intake = false;
      wristFirst = false;
      s_Photon.setAllFalse();
      shoulderPosition = -13;
      if(!flag1 && extensionPosition >= -35700){
        state = 0;
      }
      else{
        state = 1;
        flag1 = true;
      }
      extensionPosition = -35700;
      wristPosition = 61;
      turretPosition = 0;
      track = false;
      state = 0;
    }
    else if(xboxController.getRawButton(3)){
      intake = false;
      wristFirst = false;
      s_Photon.setAllFalse();
      shoulderPosition = -10;
      state = 1;
      flag1 = true;
      extensionPosition = -100;
      wristPosition = 10;
      turretPosition = 0;
      track = false;
    }
    else if(xboxController.getRawButton(2)){
      intake = false;
      wristFirst = true;
      s_Photon.setAllFalse();
      if(Intake.getConeCubeMode()){
        shoulderPosition = 0;
        if(!flag1 && extensionPosition >= -100){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -100;
        wristPosition = 1;
        turretPosition = 0;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -6;
        if(!flag1 && extensionPosition >= -100){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -100;
        wristPosition = 13;
        turretPosition = 0;
      }
      track = false;
    }
    if(xboxController.getRawAxis(2) > 0.5){
      intake = false;
      wristFirst = true;
      shoulderPosition = 0;
        if(!flag1 && extensionPosition >= -100){
          state = 0;
        }
        else{
          state = 1;
          flag1 = true;
        }
        extensionPosition = -100;
        wristPosition = 2;
        turretPosition = 0;
        track = false;
    }

    if(shoulderPosition == 0 && s_Arm.getShoulder1Position() < shoulderPosition + 1 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
      s_Arm.stopShoulder();
      flag = true;
    }
    else{
      flag = false;
    }

    manualOverride = Robot.m_robotContainer.s_Intake.getManualOverride();

    if((intake && Robot.m_robotContainer.s_Intake.intakeFull()) || (intake && manualOverride)){
      wristPosition = 2;
    }
    else if(intake && !manualOverride){
      if(Intake.getConeCubeMode()){
        wristPosition = 29;
      }
      else if(!Intake.getConeCubeMode()){
        wristPosition = 30;
      }
    }

    switch(state){
      case 0:
      if(!flag){
        s_Arm.setShoulder(shoulderPosition);
        flag1 = false;
      }
      if(wristFirst){
        s_Arm.setWrist(wristPosition);
      }
      if(s_Arm.getShoulder1Position() < shoulderPosition + 0.5 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
        if(s_Photon.getAuto() && track){
          turretPosition = s_Photon.getAutoTrackAngle();
          if(turretPosition >= -6 && turretPosition <= 6){
            s_Arm.setTurret(turretPosition);
            s_Arm.setExtension(extensionPosition);
            s_Arm.setWrist(wristPosition);
          }
          else{
            s_Arm.setTurret(0);
            s_Arm.setExtension(extensionPosition);
            s_Arm.setWrist(wristPosition);
          }
        }
        else if(!s_Photon.getAuto() || !track){
          if(turretPosition >= -6 && turretPosition <= 6){
            s_Arm.setTurret(turretPosition);
            s_Arm.setExtension(extensionPosition);
            s_Arm.setWrist(wristPosition);
          }
        }
      }
      break;

      case 1:
      if(s_Photon.getAuto() && track){
        turretPosition = s_Photon.getAutoTrackAngle();
        if(turretPosition >= -6 && turretPosition <= 6){
          s_Arm.setTurret(turretPosition);
          s_Arm.setExtension(extensionPosition);
          s_Arm.setWrist(wristPosition);
        }
        else{
          s_Arm.setTurret(0);
          s_Arm.setExtension(extensionPosition);
          s_Arm.setWrist(wristPosition);
        }
      }
      else if(!s_Photon.getAuto() || !track){
        if(turretPosition >= -6 && turretPosition <= 6){
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

  public void setFlag1False(){
    flag1 = false;
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
