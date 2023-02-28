// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntakeAuto extends CommandBase {
  private final Intake s_Intake;
  private final int speed;
  private boolean flag;
  private final Timer timer;
  /** Creates a new RunIntakeAuto. */
  public RunIntakeAuto(Intake s_Intake, int speed) {
    this.s_Intake = s_Intake;
    this.speed = speed;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Intake.intakeFull()){
      if(Intake.getConeCubeMode() && !flag){
        timer.reset();
        flag = true;
      }
      if(timer.get() > 0.25){
        s_Intake.stopIntake();
        flag = false;
      }
    }
    else if(speed == 1){
      s_Intake.runIntake(0.8);
    }
    if(speed == -1){
      s_Intake.spit();
    }
    else if(speed == 0){
      s_Intake.stopIntake();
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!s_Intake.intakeFull()){
      return true;
    }
    else{
      return false;
    }
  }
}
