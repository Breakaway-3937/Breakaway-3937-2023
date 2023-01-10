// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANdleSystem;

public class AprilTagTest extends CommandBase {
  private CANdleSystem s_CANdleSystem;
  private PhotonCamera photonCamera;
  private int tagId;
  

  /** Creates a new AprilTagTest. */
  public AprilTagTest(CANdleSystem s_CANdleSystem, PhotonCamera photonCamera) {
    this.s_CANdleSystem = s_CANdleSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_CANdleSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = photonCamera.getLatestResult();
    var target = result.getBestTarget();
    var tagId = target.getFiducialId();
    System.out.println(tagId);
    if(tagId == 1){
      System.out.print("I HAVE FOUND 1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
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
