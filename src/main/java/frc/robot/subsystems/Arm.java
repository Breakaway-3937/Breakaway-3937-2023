

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Arm extends SubsystemBase {
  private final CANSparkMax shoulder1;
  private final CANSparkMax shoulder2;
  private final WPI_TalonFX extension;
  private final CANSparkMax rotation;
  private TalonFXSensorCollection extensionEncoder;
  private double shoulder1kP, shoulder1kI, shoulder1kD, shoulder1kFF, shoulder2kP, shoulder2kI, shoulder2kD, shoulder2kFF, extensionkP, extensionkI, extensionkD, extensionkFF, rotatekP, rotatekI, rotatekD, rotatekFF;
  private RelativeEncoder shoulder1Encoder, shoulder2Encoder, rotateEncoder;
  private SparkMaxPIDController shoulder1PIDController, shoulder2PIDController, rotatePIDController;
  private final GenericEntry shoulderEncoder, extensionEncoderEntry, rotationEncoder;

  /** Creates a new Arm. */
  public Arm() {
    shoulder1 = new CANSparkMax(Constants.Arm.SHOULDER_ID, MotorType.kBrushless);
    shoulder2 = new CANSparkMax(Constants.Arm.SHOULDER_2_ID, MotorType.kBrushless);
    extension = new WPI_TalonFX(Constants.Arm.EXTENSION_ID);
    rotation = new CANSparkMax(Constants.Arm.ROTATION_ID, MotorType.kBrushless);
    setValues();
    configExtention();
    configRotation();
    configShoulder1();
    configShoulder2();
    shoulderEncoder = Shuffleboard.getTab("Arm").add("Shoulder", getShoulder1Position()).withPosition(0, 0).getEntry();
    extensionEncoderEntry = Shuffleboard.getTab("Arm").add("Extension", getExtensionPosition()).withPosition(1, 0).getEntry();
    rotationEncoder = Shuffleboard.getTab("Arm").add("Rotation", getRotationPosition()).withPosition(2, 0).getEntry();
  }

  public void positionShoulder(double position){
    shoulder1PIDController.setReference(position, ControlType.kSmartMotion);
    shoulder1.set(0.5);
  }

  public void setExtension(double position){
    extension.set(ControlMode.MotionMagic, position);
  }

  public void setRotation(double position){
    rotatePIDController.setReference(position, ControlType.kSmartMotion);
    rotation.set(0.5);
  }
  
  public double getShoulder1Position(){
    return shoulder1Encoder.getPosition();
  }

  public double getShoulder2Position(){
    return shoulder2Encoder.getPosition();
  }

  public double getExtensionPosition(){
    return extensionEncoder.getIntegratedSensorPosition();
  }

  public double getRotationPosition(){
    return rotateEncoder.getPosition();
  }

  private void configShoulder1(){
    shoulder1.restoreFactoryDefaults();
    shoulder1Encoder = shoulder1.getAlternateEncoder(Constants.Arm.ALT_ENC_TYPE, Constants.Arm.CPR);
    shoulder1PIDController = shoulder1.getPIDController();
    shoulder1PIDController.setFeedbackDevice(shoulder1Encoder);
    shoulder1Encoder.setPosition(0);

    shoulder1PIDController.setP(shoulder1kP);
    shoulder1PIDController.setI(shoulder1kI);
    shoulder1PIDController.setD(shoulder1kD);
    shoulder1PIDController.setFF(shoulder1kFF);
  }

  private void configShoulder2(){
    shoulder2.restoreFactoryDefaults();
    shoulder2Encoder = shoulder2.getEncoder();
    shoulder2PIDController = shoulder2.getPIDController();
    shoulder2PIDController.setFeedbackDevice(shoulder2Encoder);
    shoulder2Encoder.setPosition(0);

    shoulder2PIDController.setP(shoulder2kP);
    shoulder2PIDController.setI(shoulder2kI);
    shoulder2PIDController.setD(shoulder2kD);
    shoulder2PIDController.setFF(shoulder2kFF);

    shoulder2.follow(shoulder1);
  }

  private void configRotation(){
    rotation.restoreFactoryDefaults();
    rotateEncoder = rotation.getEncoder();
    rotatePIDController = rotation.getPIDController();
    rotatePIDController.setFeedbackDevice(rotateEncoder);
    rotateEncoder.setPosition(0);

    rotatePIDController.setP(rotatekP);
    rotatePIDController.setI(rotatekI);
    rotatePIDController.setD(rotatekD);
    rotatePIDController.setFF(rotatekFF);
  }

  private void configExtention(){
    extensionEncoder = extension.getSensorCollection();
    extensionEncoder.setIntegratedSensorPosition(0, 0);
    extension.config_kP(0, extensionkP);
    extension.config_kI(0, extensionkI);
    extension.config_kD(0, extensionkD);
    extension.config_kF(0, extensionkFF);
  }

  public void setValues(){
    extensionkP = 0.00005;
    extensionkI = 0;
    extensionkD = 0;
    extensionkFF = 0;
    shoulder1kP = 0;
    shoulder1kI = 0;
    shoulder1kD = 0;
    shoulder1kFF = 0;
    shoulder2kP = 0;
    shoulder2kI = 0;
    shoulder2kD = 0;
    shoulder2kFF = 0;
    rotatekP = 0;
    rotatekI = 0;
    rotatekD = 0;
    rotatekFF = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulderEncoder.setDouble(getShoulder1Position());
    extensionEncoderEntry.setDouble(getExtensionPosition());
    rotationEncoder.setDouble(getRotationPosition());
  }
}

