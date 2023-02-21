

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
  private final PhotonVision s_Photon;
  private final CANSparkMax shoulder1;
  private final CANSparkMax shoulder2;
  private final WPI_TalonFX extension;
  private final CANSparkMax rotation;
  private final CANSparkMax wristMotor;
  private TalonFXSensorCollection extensionEncoder;
  private double shoulder1kP, shoulder1kI, shoulder1kD, shoulder1kFF, shoulder2kP, shoulder2kI, shoulder2kD, shoulder2kFF, extensionkP, extensionkI, extensionkD, extensionkFF, rotatekP, rotatekI, rotatekD, rotatekFF, wristkP, wristkI, wristkD, wristkFF;
  private RelativeEncoder shoulder1Encoder, shoulder2Encoder, rotateEncoder, wristEncoder;
  private SparkMaxPIDController shoulder1PIDController, shoulder2PIDController, rotatePIDController, wristPIDController;
  private final GenericEntry shoulderEncoder, extensionEncoderEntry, rotationEncoder, wrist;

  /** Creates a new Arm. */
  public Arm(PhotonVision s_Photon) {
    this.s_Photon = s_Photon;
    shoulder1 = new CANSparkMax(Constants.Arm.SHOULDER_ID, MotorType.kBrushless);
    shoulder2 = new CANSparkMax(Constants.Arm.SHOULDER_2_ID, MotorType.kBrushless);
    extension = new WPI_TalonFX(Constants.Arm.EXTENSION_ID);
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

  public void setExtension(double position){
    extension.set(ControlMode.MotionMagic, position);
  }

  public void setRotation(double position){
    rotatePIDController.setReference(position, ControlType.kSmartMotion);
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

  public double getScoreLength(){
    if(s_Photon.closeEnough()){
      //return s_Photon.getArmStuff().getFirst() / Constants.Arm.METER_TO_FALCON;
      return 10;
    }
    else{
      return 10;
    }
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
    shoulder1PIDController.setOutputRange(-1, 1);
    shoulder1.setInverted(true);
    shoulder1.setIdleMode(IdleMode.kBrake);
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
    shoulder2PIDController.setSmartMotionMaxVelocity(500, 0);
    shoulder2PIDController.setSmartMotionMaxAccel(254, 0);
    shoulder2PIDController.setOutputRange(-1, 1);
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
    extensionEncoder = extension.getSensorCollection();
    extension.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extensionEncoder.setIntegratedSensorPosition(0, 0);
    extension.config_kP(0, extensionkP);
    extension.config_kI(0, extensionkI);
    extension.config_kD(0, extensionkD);
    extension.config_kF(0, extensionkFF);
    extension.configPeakOutputForward(1);
    extension.configPeakOutputReverse(-1);
    extension.configMotionCruiseVelocity(15000);
    extension.configMotionAcceleration(12500);
    extension.setNeutralMode(NeutralMode.Brake);
  }

  public void setValues(){
    extensionkP = 0.55;
    extensionkI = 0;
    extensionkD = 0;
    extensionkFF = 0.4;
    shoulder1kP = 6.5e-8;
    shoulder1kI = 0.5e-6;
    shoulder1kD = 0;
    shoulder1kFF = 0.0019;
    shoulder2kP = 6.5e-8;
    shoulder2kI = 0.5e-6;
    shoulder2kD = 0;
    shoulder2kFF = 0.00156;
    rotatekP = 4e-6;
    rotatekI = 0.1e-6;
    rotatekD = 0.3e-6;
    rotatekFF = 0.008;
    wristkP = 9e-6;
    wristkI = 0.1e-6;
    wristkD = 0;
    wristkFF = 0.00156;
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

