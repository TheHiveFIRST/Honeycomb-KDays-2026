// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        Optional<String> writableLogDir = waitForWritableLogDir(5, 500); // 5 attempts, 500ms apart
        if (writableLogDir.isPresent()) {
          try {
            Logger.addDataReceiver(new WPILOGWriter(writableLogDir.get()));
          } catch (Exception ex) {
            DriverStation.reportError(
                "WPILOGWriter failed to start: "
                    + ex.getClass().getSimpleName()
                    + ": "
                    + ex.getMessage(),
                false);
          }
        } else {
          DriverStation.reportWarning(
              "No writable log directory found; WPILOGWriter disabled.", false);
        }
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

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());
    StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

    // Start AdvantageKit logger
    Logger.start();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * Find a writable log directory to use for WPILOG output. Tries common roboRIO locations.
   *
   * @return an Optional containing the directory path if writable, otherwise empty
   */
  private Optional<String> findWritableLogDir() {
    // Only allow external USB (/U) locations here so we do NOT log to internal roboRIO storage
    String[] candidates = new String[] {"/U/logs", "/U"};
    for (String path : candidates) {
      try {
        File f = new File(path);
        if (!f.exists()) continue;
        if (f.isDirectory() && f.canWrite()) {
          // Use an explicit logs subfolder if available
          File logs = new File(f, "logs");
          if (logs.exists() && logs.isDirectory() && logs.canWrite()) {
            return Optional.of(logs.getAbsolutePath());
          }
          return Optional.of(f.getAbsolutePath());
        }
      } catch (Exception ignored) {
      }
    }
    return Optional.empty();
  }

  /**
   * Waits for a writable log directory to become available, retrying up to maxAttempts times with
   * delayMs between each attempt. Handles roboRIO USB mount timing issues on boot.
   */
  private Optional<String> waitForWritableLogDir(int maxAttempts, long delayMs) {
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
      Optional<String> dir = findWritableLogDir();
      if (dir.isPresent()) {
        // Extra check: actually try to create and delete a test file,
        // since canWrite() can return true on a not-yet-ready FAT32 mount
        File testFile = new File(dir.get(), ".writetest");
        try {
          if (testFile.createNewFile()) {
            testFile.delete();
            return dir; // Mount is genuinely ready
          }
        } catch (Exception ignored) {
        }
      }
      if (attempt < maxAttempts) {
        DriverStation.reportWarning(
            "USB log dir not ready (attempt " + attempt + "/" + maxAttempts + "), retrying...",
            false);
        try {
          Thread.sleep(delayMs);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          break;
        }
      }
    }
    return Optional.empty();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // robotContainer.resetSimulationField();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // SimulatedArena.getInstance().simulationPeriodic(); // TODO: DO NOT CALL WHEN RUNNING REAL
    // ROBOT

    // robotContainer.updateSimulationToAdvantageScope();
  }
}
