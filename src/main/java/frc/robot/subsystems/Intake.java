// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final CANSparkMax wristMotor;
  private final AnalogInput sensor;
  private SparkMaxPIDController wristPIDController;
  private RelativeEncoder wristEncoder;
  private double wristkP, wristkI, wristkD, wristkFF;
  private final GenericEntry distance, wrist;
  
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
    wristMotor = new CANSparkMax(Constants.Intake.WRIST_MOTOR_ID, MotorType.kBrushless);
    sensor = new AnalogInput(Constants.Intake.SENSOR_ID);
    sensor.resetAccumulator();
    setValues();
    configWristMotor();
    distance = Shuffleboard.getTab("Intake").add("Sensor", 0).withPosition(0, 0).getEntry();
    wrist = Shuffleboard.getTab("Intake").add("Wrist", getWrist()).withPosition(1, 0).getEntry();
  }
  
  public void runIntake(double speed){
    intakeMotor.set(speed);
  }

  public void runIntake(){
    intakeMotor.set(1);
  }

  public void spit(){
    intakeMotor.set(-1);
  }
  
  public void stopIntake(){
    intakeMotor.stopMotor();
  }

  public void setWrist(double position){
    wristPIDController.setReference(position, ControlType.kSmartMotion);
  }

  public double getWrist(){
    return wristEncoder.getPosition();
  }

  public boolean intakeFull(){
    if(getDistance() < 35){
      return true;
    }
    else{
      return false;
    }
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

  public void setValues(){
      wristkP = 9e-6;
      wristkI = 0.1e-6;
      wristkD = 0;
      wristkFF = 0.00156;
  }

  public double getDistance(){
    return 0.342 - 0.291 * Math.log(sensor.getVoltage());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance.setDouble(getDistance());
    wrist.setDouble(getWrist());
  }
}
