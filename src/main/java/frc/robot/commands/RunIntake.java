// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  private final Intake s_Intake;
  private final XboxController xboxController;
  private final Timer timer;
  /** Creates a new RunIntake. */
  public RunIntake(Intake s_Intake, XboxController xboxController) {
    this.s_Intake = s_Intake;
    this.xboxController = xboxController;
    timer = new Timer();
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!s_Intake.intakeFull()){
      timer.stop();
      timer.reset();
    }
    if(s_Intake.intakeFull()){
      timer.start();
    }
    if(timer.get() > 0.25){
      if(!Intake.getConeCubeMode()){
        s_Intake.stopIntake();
      }
      else if(xboxController.getRawButton(6)){
        s_Intake.runIntake();
      }
    }
    else if(xboxController.getRawButton(6)){
      s_Intake.runIntake();
    }
    if(!xboxController.getRawButton(6) && xboxController.getRawAxis(3) < 0.5){
      s_Intake.stopIntake();
    }
    else if(xboxController.getRawAxis(3) > 0.5){
      s_Intake.spit();
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
