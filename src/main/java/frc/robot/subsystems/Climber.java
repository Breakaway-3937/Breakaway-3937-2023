// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final CANSparkMax climber1, climber2;
  private RelativeEncoder climber1Encoder, climber2Encoder;
  private SparkMaxPIDController pid1, pid2;
  private double climber1kP, climber1kI, climber1kD, climber1kFF, climber2kP, climber2kI, climber2kD, climber2kFF;
  private final GenericEntry climber;

  /** Creates a new Climber. */
  public Climber() {
    climber1 = new CANSparkMax(Constants.Climber.ID_1, MotorType.kBrushless);
    climber2 = new CANSparkMax(Constants.Climber.ID_2, MotorType.kBrushless);
    setValues();
    configMotors();
    climber = Shuffleboard.getTab("Climber").add("Climber", getClimber()).withPosition(0, 0).getEntry();
  }

  public void setClimber(double position){
    pid1.setReference(position, ControlType.kSmartMotion);
    //pid2.setReference(position, ControlType.kSmartMotion);
  }

  public void runClimber(double speed){
    climber1.set(-speed);
    climber2.set(-speed);
  }

  public double getClimber(){
    return climber1Encoder.getPosition();
  }

  private void configMotors(){
    climber1.restoreFactoryDefaults();
    //climber2.restoreFactoryDefaults();
    climber1Encoder = climber1.getEncoder();
    //climber2Encoder = climber2.getEncoder();
    pid1 = climber1.getPIDController();
    //pid2 = climber2.getPIDController();
    pid1.setFeedbackDevice(climber1Encoder);
    //pid2.setFeedbackDevice(climber2Encoder);
    climber1Encoder.setPosition(0);
    //climber2Encoder.setPosition(0);
    pid1.setOutputRange(-1, 1);
    //pid2.setOutputRange(-1, 1);
    climber1.setIdleMode(IdleMode.kBrake);   
    //climber2.setIdleMode(IdleMode.kBrake); 
    climber1.setInverted(false);
    pid1.setSmartMotionMaxVelocity(Constants.Arm.MAX_VELOCITY_RAISE_ARM, 0);
    pid1.setSmartMotionMaxAccel(Constants.Arm.MAX_ACCEL_RAISE_ARM, 0);
    //pid2.setSmartMotionMaxVelocity(Constants.Arm.MAX_VELOCITY_RAISE_ARM, 0);
    //pid2.setSmartMotionMaxAccel(Constants.Arm.MAX_ACCEL_RAISE_ARM, 0);

    pid1.setP(climber1kP);
    pid1.setI(climber1kI);
    pid1.setD(climber1kD);
    pid1.setFF(climber1kFF);

    //pid2.setP(climber2kP);
    //pid2.setI(climber2kI);
    //pid2.setD(climber2kD);
    //pid2.setFF(climber2kFF);

    //climber1.follow(ExternalFollower.kFollowerDisabled, 0);
    climber2.follow(climber1, true);
  }

  public void setValues(){
    climber1kP = 9e-7;
    climber1kI = 0.5e-6;
    climber1kD = 0;
    climber1kFF = 0.00156;

    climber2kP = 6.5e-7;
    climber2kI = 0.5e-6;
    climber2kD = 0;
    climber2kFF = 0.00156;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climber.setDouble(getClimber());
    SmartDashboard.putNumber("Motor 1", climber1.getOutputCurrent());
    SmartDashboard.putNumber("Motor 2", climber2.getOutputCurrent());
    SmartDashboard.putBoolean("Follower", climber2.isFollower());
  }
}
