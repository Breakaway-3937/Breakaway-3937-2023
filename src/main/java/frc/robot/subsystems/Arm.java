

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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


public class Arm extends SubsystemBase {
  //private final PhotonVision s_Photon;
  private final CANSparkMax shoulder1;
  private final CANSparkMax shoulder2;
  private final WPI_TalonFX extension, extension1;
  private final CANSparkMax rotation;
  private final CANSparkMax wristMotor;
  private TalonFXSensorCollection extensionEncoder;
  private double shoulder1kP, shoulder1kI, shoulder1kD, shoulder1kFF, extensionkP, extensionkI, extensionkD, extensionkFF, rotatekP, rotatekI, rotatekD, rotatekFF, wristkP, wristkI, wristkD, wristkFF;
  private RelativeEncoder shoulder1Encoder, rotateEncoder, wristEncoder;
  private SparkMaxPIDController shoulder1PIDController, rotatePIDController, wristPIDController;
  private final GenericEntry shoulderEncoder, extensionEncoderEntry, rotationEncoder, wrist;

  /** Creates a new Arm. */
  public Arm() {
    //this.s_Photon = s_Photon;
    shoulder1 = new CANSparkMax(Constants.Arm.SHOULDER_ID, MotorType.kBrushless);
    shoulder2 = new CANSparkMax(Constants.Arm.SHOULDER_2_ID, MotorType.kBrushless);
    extension = new WPI_TalonFX(Constants.Arm.EXTENSION_ID);
    extension1 = new WPI_TalonFX(Constants.Arm.EXTENSION_ID_1);
    rotation = new CANSparkMax(Constants.Arm.ROTATION_ID, MotorType.kBrushless);
    wristMotor = new CANSparkMax(Constants.Intake.WRIST_MOTOR_ID, MotorType.kBrushless);
    setValues();
    configWristMotor();
    configExtention();
    configRotation();
    configShoulder1();
    configShoulder2();
    shoulderEncoder = Shuffleboard.getTab("Arm").add("Shoulder", getShoulder1Position()).withPosition(0, 0).getEntry();
    extensionEncoderEntry = Shuffleboard.getTab("Arm").add("Extension", getExtensionPosition()).withPosition(1, 0).getEntry();
    rotationEncoder = Shuffleboard.getTab("Arm").add("Rotation", getRotationPosition()).withPosition(2, 0).getEntry();
    wrist = Shuffleboard.getTab("Arm").add("Wrist", getWrist()).withPosition(3, 0).getEntry();
  }

  public void setShoulder(double position){
    shoulder1PIDController.setReference(position, ControlType.kSmartMotion);
  }

  public void stopShoulder(){
    shoulder1.set(0);
  }

  public void setExtension(double position){
    extension.set(ControlMode.MotionMagic, position);
  }

  public void stopExtension(){
    extension.stopMotor();
  }

  public void setRotation(double position){
    rotatePIDController.setReference(position, ControlType.kSmartMotion);
  }
  
  public double getShoulder1Position(){
    return shoulder1Encoder.getPosition();
  }

  public double getExtensionPosition(){
    return extensionEncoder.getIntegratedSensorPosition();
  }

  public double getRotationPosition(){
    return rotateEncoder.getPosition();
  }

  public double getScoreLength(){
    /*if(s_Photon.closeEnough()){
      //return s_Photon.getArmStuff().getFirst() / Constants.Arm.METER_TO_FALCON;
      return -20;
    }
    else{
      return -20;
    }*/
    return -20;
  }

  public void setWrist(double position){
    wristPIDController.setReference(position, ControlType.kSmartMotion);
  }

  public double getWrist(){
    return wristEncoder.getPosition();
  }

  private void configWristMotor(){
    wristMotor.restoreFactoryDefaults();
    wristEncoder = wristMotor.getEncoder();
    wristPIDController = wristMotor.getPIDController();
    wristPIDController.setFeedbackDevice(wristEncoder);
    wristEncoder.setPosition(0);
    wristMotor.setInverted(true);
    wristPIDController.setSmartMotionMaxVelocity(550, 0);
    wristPIDController.setSmartMotionMaxAccel(250, 0);
    wristPIDController.setOutputRange(-1, 1);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristPIDController.setP(wristkP);
    wristPIDController.setI(wristkI);
    wristPIDController.setD(wristkD);
    wristPIDController.setFF(wristkFF);
  }

  private void configShoulder1(){
    shoulder1.restoreFactoryDefaults();
    //shoulder1Encoder = shoulder1.getAlternateEncoder(Constants.Arm.ALT_ENC_TYPE, Constants.Arm.CPR);
    shoulder1Encoder = shoulder1.getEncoder();
    shoulder1PIDController = shoulder1.getPIDController();
    shoulder1PIDController.setFeedbackDevice(shoulder1Encoder);
    shoulder1Encoder.setPosition(0);

    shoulder1PIDController.setP(shoulder1kP);
    shoulder1PIDController.setI(shoulder1kI);
    shoulder1PIDController.setD(shoulder1kD);
    shoulder1PIDController.setFF(shoulder1kFF);
    shoulder1PIDController.setSmartMotionMaxVelocity(500, 0);
    shoulder1PIDController.setSmartMotionMaxAccel(254, 0);
    shoulder1PIDController.setOutputRange(-1, 0.75);
    shoulder1.setInverted(true);
    shoulder1.setIdleMode(IdleMode.kBrake);
  }

  private void configShoulder2(){
    shoulder2.restoreFactoryDefaults();
    shoulder2.setIdleMode(IdleMode.kBrake);
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
    rotatePIDController.setSmartMotionMaxVelocity(500, 0);
    rotatePIDController.setSmartMotionMaxAccel(250, 0);
    rotatePIDController.setOutputRange(-1, 1);
    rotation.setIdleMode(IdleMode.kBrake);
  }

  private void configExtention(){
    extension.configFactoryDefault();
    extension1.configFactoryDefault();
    extensionEncoder = extension.getSensorCollection();
    extension.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extension1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extensionEncoder.setIntegratedSensorPosition(0, 0);
    extension.config_kP(0, extensionkP);
    extension.config_kI(0, extensionkI);
    extension.config_kD(0, extensionkD);
    extension.config_kF(0, extensionkFF);
    extension.configPeakOutputForward(0.9);
    extension.configPeakOutputReverse(-0.9);
    extension.configMotionCruiseVelocity(15000);
    extension.configMotionAcceleration(12500);
    extension.setNeutralMode(NeutralMode.Brake);
    extension1.setNeutralMode(NeutralMode.Brake);
    extension1.follow(extension);
  }

  public void setValues(){
    extensionkP = 0.75;
    extensionkI = 0.0075;
    extensionkD = 7.5;
    extensionkFF = 0.3;
    shoulder1kP = 5e-8; //6.5e-8
    shoulder1kI = 0; //0.5e-6
    shoulder1kD = 0;
    shoulder1kFF = 0.001; //0.0019
    rotatekP = 4e-9;
    rotatekI = 0.1e-6;
    rotatekD = 0.3e-8;
    rotatekFF = 0.008;
    wristkP = 9e-6;
    wristkI = 0.1e-6;
    wristkD = 0;
    wristkFF = 0.0018;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulderEncoder.setDouble(getShoulder1Position());
    extensionEncoderEntry.setDouble(getExtensionPosition());
    rotationEncoder.setDouble(getRotationPosition());
    wrist.setDouble(getWrist());
  }
}

