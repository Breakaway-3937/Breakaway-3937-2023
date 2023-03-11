

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Arm extends SubsystemBase {
  private final PhotonVision s_Photon;
  private final CANSparkMax leadShoulder;
  private final CANSparkMax followerShoulder;
  private final WPI_TalonFX leadExtension, followerExtension;
  private final CANSparkMax turret;
  private final CANSparkMax wristMotor;
  private TalonFXSensorCollection extensionEncoder;
  private double shoulderkP, shoulderkI, shoulderkD, shoulderkFF, extensionkP, extensionkI, extensionkD, extensionkFF, turretkP, turretkI, turretkD, turretkFF, wristkP, wristkI, wristkD, wristkFF;
  private RelativeEncoder shoulderEncoder, turretEncoder, wristEncoder;
  private SparkMaxPIDController shoulderPIDController, turretPIDController, wristPIDController;
  private final GenericEntry shoulderEncoderEntry, extensionEncoderEntry, turretEncoderEntry, wristEncoderEntry;

  /** Creates a new Arm. */
  public Arm(PhotonVision s_Photon) {
    this.s_Photon = s_Photon;
    leadShoulder = new CANSparkMax(Constants.Arm.LEAD_SHOULDER_ID, MotorType.kBrushless);
    followerShoulder = new CANSparkMax(Constants.Arm.FOLLOWER_SHOULDER_ID, MotorType.kBrushless);
    leadExtension = new WPI_TalonFX(Constants.Arm.LEAD_EXTENSION_ID);
    followerExtension = new WPI_TalonFX(Constants.Arm.FOLLOWER_EXTENSION_ID);
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
    leadExtension.set(ControlMode.MotionMagic, position);
  }

  public void setTurret(double position){
    turretPIDController.setReference(position, ControlType.kSmartMotion);
  }

  public void runTurret(){
    double tx = -NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    if(getTurretPosition() > 4 && tx > 0){
      turret.set(0);
    }
    else if(getTurretPosition() < -4 && tx < 0){
      turret.set(0);
    }
    else{
      turret.set(tx * 0.035);
    }
    SmartDashboard.putNumber("Speed", tx * 0.035);
  }
  
  public double getShoulder1Position(){
    return shoulderEncoder.getPosition();
  }

  public double getExtensionPosition(){
    return extensionEncoder.getIntegratedSensorPosition();
  }

  public double getTurretPosition(){
    return turretEncoder.getPosition();
  }

  public double getLengthToTarget(){
    if(s_Photon.closeEnough()){
      //return s_Photon.getArmStuff().getFirst() / Constants.Arm.METER_TO_FALCON;
      return -20;
    }
    else{
      return -20;
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
    wristPIDController.setSmartMotionMaxVelocity(750, 0);
    wristPIDController.setSmartMotionMaxAccel(500, 0);
    wristPIDController.setOutputRange(-1, 1);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristPIDController.setP(wristkP);
    wristPIDController.setI(wristkI);
    wristPIDController.setD(wristkD);
    wristPIDController.setFF(wristkFF);
  }

  private void configLeadShoulder(){
    leadShoulder.restoreFactoryDefaults();
    //shoulder1Encoder = shoulder1.getAlternateEncoder(Constants.Arm.ALT_ENC_TYPE, Constants.Arm.CPR);
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
    leadShoulder.setInverted(true);
    leadShoulder.setIdleMode(IdleMode.kBrake);
  }

  private void configFollowerShoulder(){
    followerShoulder.restoreFactoryDefaults();
    followerShoulder.setIdleMode(IdleMode.kBrake);
    followerShoulder.follow(leadShoulder);
  }

  private void configTurret(){
    turret.restoreFactoryDefaults();
    turretEncoder = turret.getEncoder();
    turretPIDController = turret.getPIDController();
    turretPIDController.setFeedbackDevice(turretEncoder);
    turretEncoder.setPosition(0);

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
    leadExtension.configFactoryDefault();
    followerExtension.configFactoryDefault();
    extensionEncoder = leadExtension.getSensorCollection();
    leadExtension.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extensionEncoder.setIntegratedSensorPosition(0, 0);
    leadExtension.config_kP(0, extensionkP);
    leadExtension.config_kI(0, extensionkI);
    leadExtension.config_kD(0, extensionkD);
    leadExtension.config_kF(0, extensionkFF);
    leadExtension.configPeakOutputForward(0.9);
    leadExtension.configPeakOutputReverse(-0.9);
    leadExtension.configMotionCruiseVelocity(15000);
    leadExtension.configMotionAcceleration(12500);
    leadExtension.setNeutralMode(NeutralMode.Brake);
    followerExtension.setNeutralMode(NeutralMode.Brake);
    followerExtension.follow(leadExtension);
  }

  public void setValues(){
    extensionkP = 0.75;
    extensionkI = 0.0075;
    extensionkD = 7.5;
    extensionkFF = 0.3;
    shoulderkP = 4e-8; //6.5e-8
    shoulderkI = 0; //0.5e-6
    shoulderkD = 0;
    shoulderkFF = 0.001; //0.0019
    turretkP = 4e-9;
    turretkI = 0;
    turretkD = 0;
    turretkFF = 0.0016;
    wristkP = 7e-7;
    wristkI = 0;
    wristkD = 0;
    wristkFF = 0.001;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shoulderEncoderEntry.setDouble(getShoulder1Position());
    extensionEncoderEntry.setDouble(getExtensionPosition());
    turretEncoderEntry.setDouble(getTurretPosition());
    wristEncoderEntry.setDouble(getWrist());
  }
}

