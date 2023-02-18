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
  private final Intake s_Intake;
  private double armPosition;
  /** Creates a new RunArm. */
  public RunArm(Arm s_Arm, Joystick joystick, PhotonVision s_Photon, XboxController xboxController, Intake s_Intake) {
    this.joystick = joystick;
    this.s_Arm = s_Arm;
    this.s_Photon = s_Photon;
    this.xboxController = xboxController;
    this.s_Intake = s_Intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm, s_Photon, s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //s_Arm.setShoulder(0);
    s_Arm.setExtension(10);
    s_Intake.setWrist(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getRawButton(0)){
      if(s_Photon.getSelectedScore().get(0)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(1)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(2)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(3)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(4)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(5)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(6)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(7)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      else if(s_Photon.getSelectedScore().get(8)){
        s_Arm.setShoulder(0);
        armPosition = 0;
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
      if(s_Arm.getShoulder1Position() < armPosition + 0.2 && s_Arm.getShoulder1Position() > armPosition - 0.2){
        s_Arm.setExtension(s_Arm.getScoreLength());
      }
    }
    else if(xboxController.getPOV() == 0){
      System.out.println("up");
      s_Arm.setShoulder(0);
      s_Arm.setExtension(0);

    }
    else if(xboxController.getPOV() == 90){
      System.out.println("right");
      s_Arm.setShoulder(0);
      s_Arm.setExtension(0);

    }
    else if(xboxController.getPOV() == 180){
      System.out.println("down");
      s_Arm.setExtension(0);
      s_Arm.setExtension(0);

    }
    else if(xboxController.getRawButton(1)){
      System.out.println("a");
      if(s_Arm.getConeCubeMode() == true){
        s_Intake.setWrist(6);
        s_Arm.setShoulder(0.6);
        s_Arm.setExtension(8865);
        }
      else if(s_Arm.getConeCubeMode() == false){
          s_Intake.setWrist(13);
          s_Arm.setShoulder(-0.21);
          s_Arm.setExtension(2145);
        }    
      }
    else if(xboxController.getRawButton(4)){
      System.out.println("y");
      s_Arm.setShoulder(-15);
      s_Arm.setExtension(14845);
      s_Intake.setWrist(53);
    }
    else if(xboxController.getRawButton(3)){
      System.out.println("X");
      s_Arm.setShoulder(-15); //FIXME hit robot frame
      s_Arm.setExtension(0);
      s_Intake.setWrist(0);
    }
    else if(xboxController.getRawButton(2)){
      System.out.println("b");
      s_Arm.setShoulder(-7);
      s_Arm.setExtension(0);
      s_Intake.setWrist(8);
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
