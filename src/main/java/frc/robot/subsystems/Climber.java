// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final CANSparkMax climber1, climber2;
  private final DoubleSolenoid brake;
  private RelativeEncoder climber1Encoder, climber2Encoder;
  private SparkMaxPIDController pid1, pid2;
  private double climber1kP, climber1kI, climber1kD, climber1kFF, climber2kP, climber2kI, climber2kD, climber2kFF;
  /** Creates a new Climber. */
  public Climber() {
    climber1 = new CANSparkMax(Constants.Climber.ID_1, MotorType.kBrushless);
    climber2 = new CANSparkMax(Constants.Climber.ID_2, MotorType.kBrushless);
    brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    setValues();
    configMotors();
  }

  public void setClimber(double position){
    pid1.setReference(position, ControlType.kPosition);
    climber1.set(0.5);
    pid2.setReference(position, ControlType.kPosition);
    climber2.set(0.5);
  }

  public Pair<Double, Double> getClimber(){
    return new Pair<Double, Double>(climber1Encoder.getPosition(), climber2Encoder.getPosition());
  }

  public void brakeOn(){
    brake.set(Value.kOff);
    brake.set(Value.kForward);
  }

  public void brakeOff(){
    brake.set(Value.kOff);
    brake.set(Value.kReverse);
  }

  private void configMotors(){
    climber1.restoreFactoryDefaults();
    climber2.restoreFactoryDefaults();
    climber1Encoder = climber1.getEncoder();
    climber2Encoder = climber2.getEncoder();
    pid1 = climber1.getPIDController();
    pid2 = climber2.getPIDController();
    pid1.setFeedbackDevice(climber1Encoder);
    pid2.setFeedbackDevice(climber2Encoder);
    climber1Encoder.setPosition(0);
    climber2Encoder.setPosition(0);

    pid1.setP(climber1kP);
    pid1.setI(climber1kI);
    pid1.setD(climber1kD);
    pid1.setFF(climber1kFF);

    pid2.setP(climber2kP);
    pid2.setI(climber2kI);
    pid2.setD(climber2kD);
    pid2.setFF(climber2kFF);
  }

  public void setValues(){
    climber1kP = 0;
    climber1kI = 0;
    climber1kD = 0;
    climber1kFF = 0;
    if(Constants.COMP_BOT){
      climber2kP = 0;
      climber2kI = 0;
      climber2kD = 0;
      climber2kFF = 0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
