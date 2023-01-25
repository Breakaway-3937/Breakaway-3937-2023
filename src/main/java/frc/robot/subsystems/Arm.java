

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulder;
  private CANSparkMax secondShoulder;
  private WPI_TalonFX extension;
  private CANCoder armEncoder;
  private CANSparkMax rotation;
  private double armkP, armkI, armkD, armFF;
  private double arm2kP, arm2kI, arm2kD, arm2FF;
  private double extensionkP, extensionkI, extensionkD, extensionFF;
  private double rotatekP, rotatekI, rotatekD, rotateFF;
  private RelativeEncoder shoulderEncoder;
  private SparkMaxPIDController shoulderPIDController;
  private RelativeEncoder shoulder2Encoder;
  private SparkMaxPIDController shoulder2PIDController;
  private TalonFXSensorCollection extensionEncoder;
  private SparkMaxPIDController extensionPIDController;
  private RelativeEncoder rotateEncoder;
  private SparkMaxPIDController rotatePIDController;
  /** Creates a new Arm. */
  public Arm() {
    shoulder = new CANSparkMax(0,MotorType.kBrushless);
    secondShoulder = new CANSparkMax(0,MotorType.kBrushless);
    extension = new WPI_TalonFX(0);
    armEncoder = new CANCoder(0);
    rotation = new CANSparkMax(0,MotorType.kBrushless);
  }
  public void positionArm(double position){
    shoulder.set(position);
  }
  public void position2Arm(double position){
    secondShoulder.set(position);
  }
  public void positionExtension(double position){
    extension.set(position);
  }
  public void positionRotation(double position){
    rotation.set(position);
  }
  public double getPositionArm(){
    return shoulder.get();
  }
  public double getPositionArm2(){
    return secondShoulder.get();
  }
  public double getExtinsionArm(){
    return extension.get();
  }
  public double getRotationArm(){
    return rotation.get();
  }
  public double getArmEncoder(){
    return armEncoder.getPosition();
  }
  private void configShoulder(){
    shoulder.restoreFactoryDefaults();
    shoulderEncoder = shoulder.getEncoder();
    shoulderPIDController = shoulder.getPIDController();
    shoulderPIDController.setFeedbackDevice(shoulderEncoder);
    shoulderEncoder.setPosition(0);
    shoulder.setIdleMode(IdleMode.kCoast);

    shoulderPIDController.setP(armkP);
    shoulderPIDController.setI(armkI);
    shoulderPIDController.setD(armkD);
    shoulderPIDController.setFF(armFF);
  }
  private void configShoulder2(){
    secondShoulder.restoreFactoryDefaults();
    shoulder2Encoder = secondShoulder.getEncoder();
    shoulder2PIDController = secondShoulder.getPIDController();
    shoulder2PIDController.setFeedbackDevice(shoulder2Encoder);
    shoulder2Encoder.setPosition(0);
    secondShoulder.setIdleMode(IdleMode.kCoast);

    shoulder2PIDController.setP(arm2kP);
    shoulder2PIDController.setI(arm2kI);
    shoulder2PIDController.setD(arm2kD);
    shoulder2PIDController.setFF(arm2FF);
  }
  private void configRotate(){
    rotation.restoreFactoryDefaults();
    rotateEncoder = rotation.getEncoder();
    rotatePIDController = rotation.getPIDController();
    rotatePIDController.setFeedbackDevice(rotateEncoder);
    rotateEncoder.setPosition(0);
    rotation.setIdleMode(IdleMode.kCoast);

    rotatePIDController.setP(arm2kP);
    rotatePIDController.setI(arm2kI);
    rotatePIDController.setD(arm2kD);
    rotatePIDController.setFF(arm2FF);
  }
  private void configExtention(){
    extensionEncoder = extension.getSensorCollection();
    extensionEncoder.setIntegratedSensorPosition(0, 0);
    extension.config_kP(0, extensionkP);
    extension.config_kI(0, extensionkI);
    extension.config_kD(0, extensionkD);
    extension.config_kF(0, extensionFF);
  }
  public void setValues(){
    extensionkP = 0;
    extensionkI = 0;
    extensionkD = 0;
    extensionFF = 0;
    armkP = 0;
    armkI = 0;
    armkD = 0;
    armFF = 0;
    arm2kP = 0;
    arm2kI = 0;
    arm2kD = 0;
    arm2FF = 0;
    rotatekP = 0;
    rotatekI = 0;
    rotatekD = 0;
    rotateFF = 0;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

