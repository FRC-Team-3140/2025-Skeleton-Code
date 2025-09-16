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

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerveDrive.SwerveDriveManualControl;
import frc.robot.commands.swerveDrive.SetSwerveStates;
import frc.robot.libs.NetworkTables;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.TestRunner;
import frc.robot.subsystems.odometry.NavXSim;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

  public enum SimMode {
    SIMULATE,
    REPLAY
  }

  public final SimMode simMode = SimMode.SIMULATE;


  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final TestRunner m_testRunner;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    Logger.recordMetadata("ProjectName", "CompBot2025"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      if (simMode == SimMode.REPLAY) {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      }
    }

    Logger.start(); 
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_testRunner = TestRunner.getInstance();
    m_robotContainer = RobotContainer.getInstance();

    // Pose2d pose = Odometry.getInstance().getPose();

    // Make sure the path planner config has been setup (which happens when swerve
    // drive is initialized)
    SwerveDrive.getInstance();

    // Get Pathplanner ready for autos
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    NetworkTables.voltage_d.setDouble(RobotController.getBatteryVoltage());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_testRunner.stopAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_testRunner.stopAll();

    RobotContainer.swerveDrive.setDefaultCommand(new SetSwerveStates(RobotContainer.swerveDrive));

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    NetworkTables.state_s.setString("AUTO");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_testRunner.stopAll();

    RobotContainer.swerveDrive.setDefaultCommand(new SwerveDriveManualControl(RobotContainer.swerveDrive,
        Constants.Bot.maxChassisSpeed, Constants.Bot.maxChassisTurnSpeed, true));

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    NetworkTables.state_s.setString("TELEOP");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    m_testRunner.stopAll();

    RobotContainer.swerveDrive.setDefaultCommand(new SetSwerveStates(RobotContainer.swerveDrive));

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    try {
      Constants.PathplannerConstants.config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    NetworkTables.state_s.setString("DEV");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_testRunner.updateStates();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    final double dt = 0.02;
    final double vbus = 12.0;

    // motor free speeds (RPM)
    final double vortexFreeRPM = 6784.0; // drive (Spark Flex + NEO Vortex)
    final double neoFreeRPM = 5676.0; // steer (Spark MAX + NEO)

    for (SwerveModule module : SwerveDrive.getInstance().modules) {

      // --- DRIVE ---
      double driveDuty = module.simDriveMotor.getAppliedOutput();
      double driveRPM = driveDuty * vortexFreeRPM;
      double driveRPS = driveRPM / 60.0;
      module.simDriveMotor.iterate(driveRPM, vbus, dt);
      module.simDriveEncoder.iterate(driveRPS, dt);

      // --- STEER ---
      double turnDuty = module.simTurnMotor.getAppliedOutput();
      double turnRPM = turnDuty * neoFreeRPM;
      double turnRPS = turnRPM / 60.0;
      module.simTurnMotor.iterate(turnRPM, vbus, dt);
      module.simTurnEncoder.iterate(turnRPS, dt);

      // update custom absolute encoder (convert motor rotations → wheel degrees)
      double wheelRotations = module.simTurnMotor.getPosition() / Constants.Bot.steerGearRatio;
      double angleDeg = wheelRotations * 360.0;

      // normalize to [0, 360)
      angleDeg = ((angleDeg % 360.0) + 360.0) % 360.0;

      module.turnEncoder.setDistance(angleDeg);
    }

    // --- SIMULATED GYRO ---
    // compute chassis speeds from wheel positions
    SwerveModuleState[] positions = SwerveDrive.getInstance().getModuleStates();
    ChassisSpeeds speeds = SwerveDrive.getInstance().kinematics.toChassisSpeeds(positions);
    NavXSim.getInstance().update(speeds.omegaRadiansPerSecond, dt);
  }
}
