// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.HubStatusEnum;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private long testStartTime = 0;
  private long testPeriodMilliseconds = 3000;
  private HubStatusEnum hubStatus = HubStatusEnum.BOTH;
  private HubStatusEnum hubStatusWeCareAbout = HubStatusEnum.BOTH;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // This code must be first in the constructor to (hopefully) properly run Advantagkit
    Logger.recordMetadata("ProjectName", "2026_Mummy"); // Set a metadata value
     Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA); //

    SmartDashboard.putData(CommandScheduler.getInstance());


    // drive station camera
    UsbCamera stationCamera = CameraServer.startAutomaticCapture("StationCamera", 0);
    stationCamera.setResolution(320, 240);
    stationCamera.setFPS(30);


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // Logger.registerURCL(URCL.startExternal());
    // StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    
    // Start AdvantageKit logger
    Logger.start();
    
    m_robotContainer = new RobotContainer();

    Logger.start(); //Starts the logger ability
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    double time = DriverStation.getMatchTime();
    
    // Shifting to Hub Period 1 (2:10 -> 1:45)
    if (time <= 135 && time > 134) {
      this.hubStatus = HubStatusEnum.ODD;
      this.sendActiveHubStatus();  
      m_robotContainer.rumbleBoth(1.0);
    }
    // Shifting to Hub Period 2 (1:45 -> 1:20)
    else if (time <= 110 && time > 109) {
      this.hubStatus = HubStatusEnum.EVEN;
      this.sendActiveHubStatus();  
      m_robotContainer.rumbleBoth(1.0);
    }
    // Shifting to Hub Period 3 (1:20 -> 0:55)
    else if (time <= 85 && time > 84) {
      this.hubStatus = HubStatusEnum.ODD;
      this.sendActiveHubStatus();    
      m_robotContainer.rumbleBoth(1.0);
    }
    // Shifting to Hub Period 4 (0:55 -> 0:30)
    else if (time <= 60 && time > 59) {
      this.hubStatus = HubStatusEnum.EVEN;
      this.sendActiveHubStatus();    
      m_robotContainer.rumbleBoth(1.0);
    }
    // Shifting to Hub Period Both (0:30 -> 0:00)
    else if (time <= 35 && time > 34) {
      this.hubStatus = HubStatusEnum.BOTH;
      this.sendActiveHubStatus();    
      m_robotContainer.rumbleBoth(1.0);
    }
    else {
      m_robotContainer.rumbleBoth(0.0);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    if (m_robotContainer.intake.getIntakeAngleSetpoint() == Constants.IntakeConstants.intakePivotPulseUpAngleDegrees ){
      CommandScheduler.getInstance().schedule(m_robotContainer.intake.intakePivotToAngle(Constants.IntakeConstants
      .intakePivotDeployAngleDegrees));
        }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.hood.removeDefaultCommand();
    m_robotContainer.indexer.removeDefaultCommand();
    m_robotContainer.kicker.removeDefaultCommand();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    this.hubStatus = HubStatusEnum.BOTH;
    this.sendActiveHubStatus(); 
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.hood.setDefaultCommand(m_robotContainer.hood.retractHood());
    m_robotContainer.kicker.setDefaultCommand(m_robotContainer.kicker.stopKicker());
    m_robotContainer.indexer.setDefaultCommand(m_robotContainer.indexer.stopIndexer());
    m_robotContainer.intake.setDefaultCommand(m_robotContainer.intake.intakeStopRollers());
    m_robotContainer.shooter.setDefaultCommand(m_robotContainer.shooter.shooterTurnOff());
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (DriverStation.getGameSpecificMessage().contains("B")) {
        this.hubStatusWeCareAbout = HubStatusEnum.EVEN;
      } else {
        this.hubStatusWeCareAbout = HubStatusEnum.ODD;
      } 
    } else {
      if (DriverStation.getGameSpecificMessage().contains("R")) {
        this.hubStatusWeCareAbout = HubStatusEnum.EVEN;
      } else {
        this.hubStatusWeCareAbout = HubStatusEnum.ODD;
      } 
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    this.testStartTime = System.currentTimeMillis();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (System.currentTimeMillis() > this.testStartTime + testPeriodMilliseconds) {
      // Do Command
      CommandScheduler.getInstance().schedule(
        Commands.parallel(
        this.m_robotContainer.intake.intakePivotToAngle(140)
        )
      );
      
    }

    if (System.currentTimeMillis() > this.testStartTime + testPeriodMilliseconds * 2) {
      // Do Stop Command
      CommandScheduler.getInstance().schedule(
        Commands.parallel(
        this.m_robotContainer.intake.intakePivotToAngle(0)
        )
      );
      
      // Reset timer
      this.testStartTime = System.currentTimeMillis();
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void sendActiveHubStatus() {
    if (this.hubStatus == this.hubStatusWeCareAbout || this.hubStatus == HubStatusEnum.BOTH) {
      Logger.recordOutput("GoalActive", true);
    } else {
      Logger.recordOutput("GoalActive", false);
    }
  }
}
