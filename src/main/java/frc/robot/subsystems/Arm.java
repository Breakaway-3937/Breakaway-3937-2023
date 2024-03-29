

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private final CANSparkMax leadShoulder;
  private final CANSparkMax followerShoulder;
  private final TalonFX leadExtension, followerExtension;
  private final CANSparkMax turret;
  private final CANSparkMax wristMotor;
  private TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
  private double shoulderkP, shoulderkI, shoulderkD, shoulderkFF, extensionkP, extensionkI, extensionkD, turretkP, turretkI, turretkD, turretkFF, wristkP, wristkI, wristkD, wristkFF;
  private RelativeEncoder shoulderEncoder, turretEncoder, wristEncoder;
  private SparkMaxPIDController shoulderPIDController, turretPIDController, wristPIDController;
  private final GenericEntry shoulderEncoderEntry, extensionEncoderEntry, turretEncoderEntry, wristEncoderEntry;

  /** Creates a new Arm. */
  public Arm() {
    leadShoulder = new CANSparkMax(Constants.Arm.LEAD_SHOULDER_ID, MotorType.kBrushless);
    followerShoulder = new CANSparkMax(Constants.Arm.FOLLOWER_SHOULDER_ID, MotorType.kBrushless);
    leadExtension = new TalonFX(Constants.Arm.LEAD_EXTENSION_ID);
    followerExtension = new TalonFX(Constants.Arm.FOLLOWER_EXTENSION_ID);
    turret = new CANSparkMax(Constants.Arm.TURRET_ID, MotorType.kBrushless);
    wristMotor = new CANSparkMax(Constants.Intake.WRIST_MOTOR_ID, MotorType.kBrushless);
    setValues();
    configWristMotor();
    configExtention();
    configTurret();
    configLeadShoulder();
    configFollowerShoulder();
    shoulderEncoderEntry = Shuffleboard.getTab("Arm").add("Shoulder", getShoulder1Position()).withPosition(0, 0).getEntry();
    extensionEncoderEntry = Shuffleboard.getTab("Arm").add("Extension", getExtensionPosition()).withPosition(1, 0).getEntry();
    turretEncoderEntry = Shuffleboard.getTab("Arm").add("Rotation", getTurretPosition()).withPosition(2, 0).getEntry();
    wristEncoderEntry = Shuffleboard.getTab("Arm").add("Wrist", getWrist()).withPosition(3, 0).getEntry();
  }

  public void setShoulder(double position){
    shoulderPIDController.setReference(position, ControlType.kSmartMotion);
  }

  public void stopShoulder(){
    leadShoulder.stopMotor();
  }

  public void setExtension(double position){
    leadExtension.setControl(new MotionMagicDutyCycle(position, true, 0, 0, false));
  }

  public void setTurret(double position){
    turretPIDController.setReference(position, ControlType.kSmartMotion);
  }
  
  public double getShoulder1Position(){
    return shoulderEncoder.getPosition();
  }

  public double getExtensionPosition(){
    return leadExtension.getRotorPosition().getValue();
  }

  public double getTurretPosition(){
    return turretEncoder.getPosition();
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
    wristPIDController.setSmartMotionMaxVelocity(2500, 0);
    wristPIDController.setSmartMotionMaxAccel(1000, 0);
    wristPIDController.setOutputRange(-1, 1);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setSmartCurrentLimit(40);

    wristPIDController.setP(wristkP);
    wristPIDController.setI(wristkI);
    wristPIDController.setD(wristkD);
    wristPIDController.setFF(wristkFF);
  }

  private void configLeadShoulder(){
    leadShoulder.restoreFactoryDefaults();
    shoulderEncoder = leadShoulder.getEncoder();
    shoulderPIDController = leadShoulder.getPIDController();
    shoulderPIDController.setFeedbackDevice(shoulderEncoder);
    shoulderEncoder.setPosition(0);

    shoulderPIDController.setP(shoulderkP);
    shoulderPIDController.setI(shoulderkI);
    shoulderPIDController.setD(shoulderkD);
    shoulderPIDController.setFF(shoulderkFF);
    shoulderPIDController.setSmartMotionMaxVelocity(750, 0);
    shoulderPIDController.setSmartMotionMaxAccel(350, 0);
    shoulderPIDController.setOutputRange(-1, 0.75);
    leadShoulder.setSmartCurrentLimit(60);
    leadShoulder.setInverted(true);
    leadShoulder.setIdleMode(IdleMode.kBrake);
  }

  private void configFollowerShoulder(){
    followerShoulder.restoreFactoryDefaults();
    followerShoulder.setSmartCurrentLimit(60);
    followerShoulder.setIdleMode(IdleMode.kBrake);
    followerShoulder.follow(leadShoulder);
  }

  private void configTurret(){
    turret.restoreFactoryDefaults();
    turretEncoder = turret.getEncoder();
    turretPIDController = turret.getPIDController();
    turretPIDController.setFeedbackDevice(turretEncoder);
    turretEncoder.setPosition(0);
    turret.setSmartCurrentLimit(40);

    turretPIDController.setP(turretkP);
    turretPIDController.setI(turretkI);
    turretPIDController.setD(turretkD);
    turretPIDController.setFF(turretkFF);
    turretPIDController.setSmartMotionMaxVelocity(500, 0);
    turretPIDController.setSmartMotionMaxAccel(250, 0);
    turretPIDController.setOutputRange(-1, 1);
    turret.setIdleMode(IdleMode.kBrake);
  }

  private void configExtention(){
    leadExtension.getConfigurator().apply(new TalonFXConfiguration());
    followerExtension.getConfigurator().apply(new TalonFXConfiguration());
    extensionConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    extensionConfig.Slot0.kP = extensionkP;
    extensionConfig.Slot0.kI = extensionkI;
    extensionConfig.Slot0.kD = extensionkD;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 73;
    extensionConfig.MotionMagic.MotionMagicAcceleration = 61;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = 35;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.SupplyCurrentThreshold = 50;
    extensionConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Audio.AllowMusicDurDisable = true;
    leadExtension.getConfigurator().apply(extensionConfig);
    followerExtension.getConfigurator().apply(extensionConfig);
    followerExtension.setControl(new Follower(Constants.Arm.LEAD_EXTENSION_ID, false));
  }

  public void setValues(){
    extensionkP = 1.501466275659824;
    extensionkI = 0;
    extensionkD = 0.0150146;
    shoulderkP = 4e-8;
    shoulderkI = 0;
    shoulderkD = 0;
    shoulderkFF = 0.001;
    turretkP = 4e-8;
    turretkI = 0;
    turretkD = 0;
    turretkFF = 0.001;
    wristkP = 5e-7;
    wristkI = 0;
    wristkD = 0;
    wristkFF = 0.0005;
  }

  public TalonFX getExtension(){
    return leadExtension;
  }

  public TalonFX getExtension1(){
    return followerExtension;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulderEncoderEntry.setDouble(getShoulder1Position());
    extensionEncoderEntry.setDouble(getExtensionPosition());
    turretEncoderEntry.setDouble(getTurretPosition());
    wristEncoderEntry.setDouble(getWrist());
    Logger.getInstance().recordOutput("Shoulder", getShoulder1Position());
    Logger.getInstance().recordOutput("Extension", getExtensionPosition());
    Logger.getInstance().recordOutput("Turret", getTurretPosition());
    Logger.getInstance().recordOutput("Wrist", getWrist());
  }
}

