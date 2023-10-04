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
    extensionPosition = -0.0146484375;
    wristPosition = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(level == 3 && !flag1){
      if(Intake.getConeCubeMode()){
        shoulderPosition = -13;
        extensionPosition = -21.728515625;
        wristPosition = 60;
        turretPosition = 0;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -11.25;
        extensionPosition = -21.728515625;
        wristPosition = 44.2;
        turretPosition = 0;
      }
      state = 0;
    }
    else if(level == 2 && !flag1){
      if(Intake.getConeCubeMode()){
        shoulderPosition = -13.75;
        if(extensionPosition > -9.521484375){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -9.521484375;
        wristPosition = 66.5;
        turretPosition = 0;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = -10.75;
        if(!flag1 && extensionPosition > -9.27734375){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -9.27734375;
        wristPosition = 44.2;
        turretPosition = 0;
      }
      flag1 = true;
    }
    else if(level == 1 && !flag1){
      shoulderPosition = 0;
      if(extensionPosition > -0.048828125){
        state = 0;
      }
      else{
        state = 1;
      }
      extensionPosition = -0.048828125;
      wristPosition = 13.5;
      turretPosition = 0;
      flag1 = true;
    }
    if(level == -1 && !flag1){
      if(Intake.getConeCubeMode()){
        shoulderPosition = 0;
        if(extensionPosition > -0.048828125){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -0.048828125;
        wristPosition = 29;
        turretPosition = 0;
      }
      else if(!Intake.getConeCubeMode()){
        shoulderPosition = 0;
        if(extensionPosition > -0.048828125){
          state = 0;
        }
        else{
          state = 1;
        }
        extensionPosition = -0.048828125;
        wristPosition = 30;
        turretPosition = 0;
      }
      flag1 = true;
    }
    else if(level == 0 && !flag1){
      shoulderPosition = -10;
      state = 1;
      extensionPosition = -0.048828125;
      wristPosition = 10;
      turretPosition = 0;
      flag1 = true;
    }

  if(shoulderPosition == 0 && s_Arm.getShoulder1Position() < shoulderPosition + 1 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
    s_Arm.stopShoulder();
    flag = true;
  }
  else{
    flag = false;
  }

  switch(state){
    case 0:
      if(!flag){
        s_Arm.setShoulder(shoulderPosition);
      }
      if(s_Arm.getShoulder1Position() < shoulderPosition + 1 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
          s_Arm.setTurret(turretPosition);
          s_Arm.setExtension(extensionPosition);
          s_Arm.setWrist(wristPosition);
      }
      if(s_Arm.getExtensionPosition() < extensionPosition + 0.06103515625 && s_Arm.getExtensionPosition() > extensionPosition - 0.06103515625 && s_Arm.getShoulder1Position() < shoulderPosition + 1 && s_Arm.getShoulder1Position() > shoulderPosition - 0.5){
        done = true;
      }
      break;

      case 1:
      s_Arm.setTurret(turretPosition);
      s_Arm.setExtension(extensionPosition);
      s_Arm.setWrist(wristPosition);
      if(s_Arm.getExtensionPosition() < extensionPosition + 0.06103515625 && s_Arm.getExtensionPosition() > extensionPosition - 0.06103515625 && !flag){
        s_Arm.setShoulder(shoulderPosition);
        flag2 = true;
      }
      if(flag){
        flag2 = true;
      }
      if(s_Arm.getExtensionPosition() < extensionPosition + 0.06103515625 && s_Arm.getExtensionPosition() > extensionPosition - 0.06103515625 && flag2){
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
