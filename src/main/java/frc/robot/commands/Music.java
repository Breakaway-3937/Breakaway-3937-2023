// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Music extends CommandBase {
  private Drivetrain s_Drivetrain;
  private Arm s_Arm;
  private Intake s_Intake;
  private Orchestra orchestra;
  private int count = 10;
  private double time = 0;
  /** Creates a new Music. */
  public Music(Drivetrain s_Drivetrain, Arm s_Arm, Intake s_Intake) {
    this.s_Drivetrain = s_Drivetrain;
    this.s_Arm = s_Arm;
    this.s_Intake = s_Intake;
    orchestra = new Orchestra();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Drivetrain, s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(int i = 0; i < 4; i++){
      orchestra.addInstrument(s_Drivetrain.swerveMods[i].getDriveMotor());
      orchestra.addInstrument(s_Drivetrain.swerveMods[i].getAngleMotor());
    }
    orchestra.addInstrument(s_Arm.getExtension());
    orchestra.addInstrument(s_Arm.getExtension1());
    orchestra.addInstrument(s_Intake.getIntake());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(orchestra.getCurrentTime() >= time){
      if(count == 0){
        orchestra.loadMusic("GodBless.chrp");
        count++;
        time = 99;
        orchestra.play();
      }
      else if(count == 1){
        orchestra.loadMusic("Champions.chrp");
        count++;
        time = 180;
        orchestra.play();
      }
      else if(count == 2){
        orchestra.loadMusic("Giorno.chrp");
        count++;
        time = 285;
        orchestra.play();
      }
      else if(count == 3){
        orchestra.loadMusic("Duel.chrp");
        count++;
        time = 250;
        orchestra.play();
      }
      else if(count == 4){
        orchestra.loadMusic("Imperial.chrp");
        count++;
        time = 106;
        orchestra.play();
      }
      else if(count == 5){
        orchestra.loadMusic("Megalovania.chrp");
        count++;
        time = 38;
        orchestra.play();
      }
      else if(count == 6){
        orchestra.loadMusic("Mountain.chrp");
        count++;
        time = 151;
        orchestra.play();
      }
      else if(count == 7){
        orchestra.loadMusic("Pomp.chrp");
        count++;
        time = 107;
        orchestra.play();
      }
      else if(count == 8){
        orchestra.loadMusic("US.chrp");
        count++;
        time = 219;
        orchestra.play();
      }
      else if(count == 9){
        orchestra.loadMusic("76.chrp");
        count++;
        time = 200;
        orchestra.play();
      }
      if(count == 10){
        orchestra.loadMusic("Elves.chrp");
        time = 42;
        orchestra.play();
      }
      if(count == 10){
        count = 0;
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
