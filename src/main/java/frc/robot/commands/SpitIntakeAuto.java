// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SpitIntakeAuto extends CommandBase {
  private final Intake s_Intake;
  private boolean flag;
  private final Timer timer;
  /** Creates a new SpitIntakeAuto. */
  public SpitIntakeAuto(Intake s_Intake) {
    this.s_Intake = s_Intake;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
    s_Intake.spit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!s_Intake.intakeFull() && !flag){
      timer.reset();
      timer.start();
      flag = true;
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
    if(!s_Intake.intakeFull() && timer.get() > 0.25){
      return true;
    }
    else{
      return false;
    }
  }
}
