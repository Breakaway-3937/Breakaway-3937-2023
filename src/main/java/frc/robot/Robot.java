// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;

  private PowerDistribution powerDistribution;

  private GenericEntry canUtil = Shuffleboard.getTab("System").add("CanUtil", 0).withPosition(0, 0).getEntry();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    if (isReal()) {
      Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      powerDistribution = new PowerDistribution(25, ModuleType.kRev); // Enables power distribution logging
    }
    else{
      powerDistribution = new PowerDistribution(25, ModuleType.kRev);
    }

    if(Constants.DEBUG){
      Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    }
    
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    powerDistribution.setSwitchableChannel(true);
    powerDistribution.clearStickyFaults();
    CommandScheduler.getInstance().setPeriod(0.025);
    ComplexWidget pdh = Shuffleboard.getTab("System").add("PDH", powerDistribution).withPosition(1, 0);
    pdh.hashCode();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * ShuffleBoard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    canUtil.setDouble(RobotController.getCANStatus().percentBusUtilization * 100);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().schedule(m_robotContainer.c_Music.ignoringDisable(true));
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.s_Arm.setDefaultCommand(m_robotContainer.c_RunArm);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}






















































































/*
 * 2(pi^2(240i)) = (-(x))9
 */













/*
 * Beavers to inches = 36 / number of beaver
 */











/*if(DriverStation.isDisabled() && flag && DriverStation.getMatchTime() > 130){
  //FIX ME Get RoboRandy and Greyson to Approve!
  flag = false;
  Shuffleboard.getTab("GREYSON").add("GREYSON", "STRONGER CORE!!  MORE SIX-PACK!! BEST LIFT!!").withPosition(0, 0);
  Shuffleboard.selectTab("GREYSON");
}
if(DriverStation.isEStopped() && flag1){
  //FIX ME Get RoboRandy to Approve!
  flag1 = false;
  Shuffleboard.getTab("Death").add("Death", "We have failed!!! :(").withPosition(0, 0);
  Shuffleboard.getTab("Death").add("Death2", "MARKKKKKKKK!!!!!!").withPosition(0, 1);
  Shuffleboard.selectTab("Death");
}*/
//This is the life of a programmer. We are under valued and over worked. Everything we get breaks. There are so many mechanical issues with the robot that I have lost count. This will probably never get seen again, but if it does, good luck in your future in Java. I hope that you don't have to use Shuffleboard.
//Sincerely, 
//  Jack Left