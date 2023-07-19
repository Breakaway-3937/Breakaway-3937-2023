// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Climber extends SubsystemBase {
  private final CANSparkMax climber1, climber2;
  private RelativeEncoder climber1Encoder;
  private SparkMaxPIDController pid1;
  private double climber1kP, climber1kI, climber1kD, climber1kFF;
  private final GenericEntry climber;

  /** Creates a new Climber. */
  public Climber() {
    climber1 = new CANSparkMax(Constants.Climber.LEAD_ID, MotorType.kBrushless);
    climber2 = new CANSparkMax(Constants.Climber.FOLLOWER_ID, MotorType.kBrushless);
    setValues();
    configMotors();
    climber = Shuffleboard.getTab("Climber").add("Climber", getClimber()).withPosition(0, 0).getEntry();
  }

  public void setClimber(double position){
    pid1.setReference(position, ControlType.kSmartMotion);
  }

  public double getClimber(){
    return climber1Encoder.getPosition();
  }

  private void configMotors(){
    climber1.restoreFactoryDefaults();
    climber2.restoreFactoryDefaults();
    climber1Encoder = climber1.getEncoder();
    pid1 = climber1.getPIDController();
    pid1.setFeedbackDevice(climber1Encoder);
    climber1Encoder.setPosition(0);
    pid1.setOutputRange(-1, 1);
    climber1.setIdleMode(IdleMode.kBrake);   
    climber1.setInverted(false);
    pid1.setSmartMotionMaxVelocity(3000, 0);
    pid1.setSmartMotionMaxAccel(1500, 0);
    pid1.setP(climber1kP);
    pid1.setI(climber1kI);
    pid1.setD(climber1kD);
    pid1.setFF(climber1kFF);

    
    climber2.setIdleMode(IdleMode.kBrake);   
    climber2.follow(climber1, true);
  }

  public void setFastClimberSpeed(){
    pid1.setSmartMotionMaxVelocity(3000, 0);
    pid1.setSmartMotionMaxAccel(1500, 0);
  }

  public void setSlowClimberSpeed(){
    pid1.setSmartMotionMaxVelocity(500, 0);
    pid1.setSmartMotionMaxAccel(250, 0);
  }

  public void setValues(){
    climber1kP = 1e-10;
    climber1kI = 0;
    climber1kD = 0;
    climber1kFF = 0.001;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climber.setDouble(getClimber());
    Logger.getInstance().recordOutput("Climber", getClimber());
  }
}