// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
* @author Jeffords
*/
public class Intake extends SubsystemBase {
  /**
* @author Jeffords
*/
  private WPI_TalonSRX intkateMotorTop;
  private WPI_TalonSRX intkateMotorBottom;
  private CANSparkMax wristMotor;
  private AnalogInput rangeFinder;
  private DoubleSolenoid doubleSolenoid; //FIXME Rename
  private SparkMaxPIDController wristPIDConsController;
  private RelativeEncoder wristEncoder;
  private double wristkP, wristkI, wristkD, wristFF;
  
  /** Creates a new Intake.
   * @author Jeffords
   */
  public Intake() {
    intkateMotorTop = new WPI_TalonSRX(Constants.Intake.INTKAE_MOTOR_TOP);
    intkateMotorBottom = new WPI_TalonSRX(Constants.Intake.INTKAE_MOTOR_BOTTOM);
    wristMotor = new CANSparkMax(Constants.Intake.WRIST_MOTOR_ID, MotorType.kBrushless);
    rangeFinder = new AnalogInput(Constants.Intake.RANGE_FINDER_ID);
    doubleSolenoid = new DoubleSolenoid(Constants.Intake.DOUBLE_SOLENOID_ID, PneumaticsModuleType.CTREPCM, 0, 0); //FIXME
    configWristMotor();
  }
  /**
   * Run intake
   * @author Jeffords
   */
  public void runIntake(double speed){
    intkateMotorBottom.set(TalonSRXControlMode.Velocity, speed);
    intkateMotorTop.set(TalonSRXControlMode.Velocity, speed);
  }

  /**
   * Stop intake
   * @author Jeffords
   */
  public void stopIntake(){
    intkateMotorBottom.stopMotor();
    intkateMotorTop.stopMotor();
  }
  /**
   * Config wrist motor PID
   * @author Jeffords
   */
  private void configWristMotor(){
    wristMotor.restoreFactoryDefaults();
    wristEncoder = wristMotor.getEncoder();
    wristPIDConsController = wristMotor.getPIDController();
    wristPIDConsController.setFeedbackDevice(wristEncoder);
    wristEncoder.setPosition(0);

    wristPIDConsController.setP(wristkP);
    wristPIDConsController.setI(wristkI);
    wristPIDConsController.setD(wristkD);
    wristPIDConsController.setFF(wristFF);
  }

  /**
   * Set Values of Wrist PID
   * @author Jeffords
   */
  public void setValues(){
    wristkP = 0;
    wristkI = 0;
    wristkD = 0;
    wristFF = 0;
    if(Constants.COMP_BOT){
    wristkP = 0;
    wristkI = 0;
    wristkD = 0;
    wristFF = 0;
    }
  
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
