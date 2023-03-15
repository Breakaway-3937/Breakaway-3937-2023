// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RunArmAuto extends CommandBase {
  private boolean flag, flag1, flag2, done = false;
  private int state, level;
  private final Arm s_Arm;
  private double shoulderPosition, turretPosition, extensionPosition, wristPosition = 0;
  /** Creates a new RunArmAuto. */
  public RunArmAuto(Arm s_Arm, int level) {
    this.s_Arm = s_Arm;
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    flag = false;
    flag1 = false;
    flag2 = false;
    shoulderPosition = -10;
    turretPosition = 0; 
    extensionPosition = -20;
    wristPosition = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(level == 3 && !flag2){
      shoulderPosition = -13;
      extensionPosition = -46500;
      wristPosition = 36;
      turretPosition = 0;
      state = 0;
    }
    else if(level == 2 && !flag2){
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
      flag2 = true;
    }
    else if(level == 1 && !flag2){
      shoulderPosition = -5;
      if(extensionPosition > -307){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -307;
      wristPosition = 35;
      flag2 = true;
    }
    if(level == -1 && !flag2){
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
        flag2 = true;
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
        wristPosition = 12.45;
        turretPosition = 0;
        flag2 = true;
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
        flag2 = true;
      }   
    }
    else if(level == 0 && !flag2){
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
      flag2 = true;
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
      if(s_Arm.getTurretPosition() < turretPosition + 0.5 && s_Arm.getTurretPosition() > turretPosition - 0.5 && flag){
        s_Arm.setExtension(extensionPosition);
        s_Arm.setWrist(wristPosition);
        flag = false;
      }
      if(s_Arm.getExtensionPosition() < extensionPosition + 125 && s_Arm.getExtensionPosition() > extensionPosition - 125 && s_Arm.getWrist() < wristPosition + 1 && s_Arm.getWrist() > wristPosition - 1){
        done = true;
      }
      break;
    case 1: 
      s_Arm.setWrist(wristPosition);
      s_Arm.setExtension(extensionPosition);
      if(s_Arm.getExtensionPosition() < extensionPosition + 125 && s_Arm.getExtensionPosition() > extensionPosition - 125){
        //s_Arm.setRotation(turretPosition);
        flag = true;
      }
      if(s_Arm.getTurretPosition() < turretPosition + 0.5 && s_Arm.getTurretPosition() > turretPosition - 0.5 && flag){
        s_Arm.setShoulder(shoulderPosition);
        flag = false;
      }
      if(s_Arm.getShoulder1Position() < shoulderPosition + 0.5 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
        done = true;
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
    return done;
  }
}
