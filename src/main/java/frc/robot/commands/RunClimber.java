// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {
  private final Climber s_Climber;
  private final XboxController xboxController;
 
  /** Creates a new RunClimber. */
  public RunClimber(Climber s_Climber, XboxController xboxController){
    this.xboxController = xboxController;
    this.s_Climber = s_Climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getRawButton(5)){
      s_Climber.setClimber(42);
    }
    else if(xboxController.getRawAxis(2) > 0.5){
      s_Climber.setClimberSpeed();
      s_Climber.setClimber(-5); 
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
