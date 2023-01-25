

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulder;
  private CANSparkMax secondShoulder;
  private WPI_TalonFX extension;
  private CANCoder armEncoder;
  private CANSparkMax rotation;
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

