// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor;
  private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
  private final AnalogInput uSSensor, bBSensor;
  private final GenericEntry usDistance, bBDistance;
  private static boolean cone;
  private boolean override;

  public Intake() {
    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
    intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 25;
    intakeMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    intakeMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotorConfig.Audio.AllowMusicDurDisable = true;
    intakeMotor.getConfigurator().apply(intakeMotorConfig);
    uSSensor = new AnalogInput(Constants.Intake.US_SENSOR_ID);
    uSSensor.resetAccumulator();
    bBSensor = new AnalogInput(Constants.Intake.BB_SENSOR_ID);
    bBSensor.resetAccumulator();
    usDistance = Shuffleboard.getTab("Intake").add("US Sensor", 0).withPosition(0, 0).getEntry();
    bBDistance = Shuffleboard.getTab("Intake").add("BB Sensor", 0).withPosition(1, 0).getEntry();
  }

  public void runIntake(){
    if(getConeCubeMode()){
      intakeMotor.setControl(new DutyCycleOut(-1));
    }
    else{
      intakeMotor.setControl(new DutyCycleOut(1));
    }
  }

  public void spit(){
    if(getConeCubeMode()){
      intakeMotor.setControl(new DutyCycleOut(1));
    }
    else{
      intakeMotor.setControl(new DutyCycleOut(-0.8));
    }
  }
  
  public void stopIntake(){
    intakeMotor.setControl(new DutyCycleOut(0));
  }

  public boolean intakeFull(){
    if(getConeCubeMode()){
      if(getDistance() < 0.3){
        return true;
      }
      else{
        return false;
      }
    }
    else{
      if(bBSensor.getValue() > 4000){
        return true;
      }
      else{
        return false;
      }
    }
  }

  public void setManualOverride(boolean override){
    this.override = override;
  }

  public boolean getManualOverride(){
    return override;
  }

  public void setCone(){
    cone = true;
  }

  public void setCube(){
    cone = false;
  }

  public static boolean getConeCubeMode(){
    return cone;
  }

  public double getDistance(){
    return -0.261 * Math.log(uSSensor.getVoltage()) + 0.3455;
  }

  public TalonFX getIntake(){
    return intakeMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    usDistance.setDouble(getDistance());
    bBDistance.setDouble(bBSensor.getValue());
    Logger.getInstance().recordOutput("UltraSonic", getDistance());
    Logger.getInstance().recordOutput("BeamBreak", bBSensor.getValue());
  }
}
