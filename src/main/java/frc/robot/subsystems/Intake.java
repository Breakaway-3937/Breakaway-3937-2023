// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private WPI_TalonSRX intkateMotorTop;
  private WPI_TalonSRX intkateMotorBottom;
  private CANSparkMax wristMotor;
  private AnalogInput sensor;
  private DoubleSolenoid clamp;
  private SparkMaxPIDController wristPIDController;
  private RelativeEncoder wristEncoder;
  private double wristkP, wristkI, wristkD, wristkFF;
  
  public Intake() {
    intakeTop = new WPI_TalonSRX(Constants.Intake.INTKAE_MOTOR_TOP);
    intakeBottom = new WPI_TalonSRX(Constants.Intake.INTKAE_MOTOR_BOTTOM);
    wristMotor = new CANSparkMax(Constants.Intake.WRIST_MOTOR_ID, MotorType.kBrushless);
    sensor = new AnalogInput(Constants.Intake.RANGE_FINDER_ID);
    clamp = new DoubleSolenoid(Constants.PCM_ID, PneumaticsModuleType.CTREPCM, 0, 1);
    configWristMotor();
  }
  
  public void runIntake(double speed){
    intakeBottom.set(TalonSRXControlMode.Velocity, speed);
    intakeTop.set(TalonSRXControlMode.Velocity, speed);
  }
  
  public void stopIntake(){
    intakeBottom.stopMotor();
    intakeTop.stopMotor();
  }

  public void clampOn(){
    clamp.set(Value.kOff);
    clamp.set(Value.kForward);
  }

  public void clampOff(){
    clamp.set(Value.kOff);
    clamp.set(Value.kReverse);
  }

  public void setWrist(double position){
    wristPIDController.setReference(position, ControlType.kPosition);
    wristMotor.set(0.5);
  }
  
  private void configWristMotor(){
    wristMotor.restoreFactoryDefaults();
    wristEncoder = wristMotor.getEncoder();
    wristPIDController = wristMotor.getPIDController();
    wristPIDController.setFeedbackDevice(wristEncoder);
    wristEncoder.setPosition(0);

    wristPIDController.setP(wristkP);
    wristPIDController.setI(wristkI);
    wristPIDController.setD(wristkD);
    wristPIDController.setFF(wristkFF);
  }

  public void setValues(){
      wristkP = 0;
      wristkI = 0;
      wristkD = 0;
      wristkFF = 0;
    if(Constants.COMP_BOT){
      wristkP = 0;
      wristkI = 0;
      wristkD = 0;
      wristkFF = 0;
    }
  
  }

  public boolean intakeFull(){
    if(sensor.getValue() > 2000){
      return true;
    }
    else{
      return false;
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
