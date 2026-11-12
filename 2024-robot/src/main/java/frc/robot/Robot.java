// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private BooleanSubscriber cameraIsLogging;
  private BooleanPublisher isLogging;

  private static final String logPath = "/U/logs";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    var ntInstance = NetworkTableInstance.getDefault();
    isLogging = ntInstance.getBooleanTopic("/Vision/IsLogging").publish();
    isLogging.set(false);

    // Pi is logging
    cameraIsLogging = ntInstance.getBooleanTopic("Vision/PiIsLogging").subscribe(false);

    // record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter(logPath));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // replaying a log, set up replay source
        setUseTiming(false); // run as fast as possible
        final var logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // see http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit
    // Logger.disableDeterministicTimestamps()

    // start AdvantageKit logger
    Logger.start();

    startCommandLogging();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    robotContainer.updateVisionTracking();
  }

  @Override
  public void disabledInit() {
    robotContainer.onDisable();
  }

  @Override
  public void disabledPeriodic() {
    if (cameraIsLogging.get()) {
      robotContainer.setLoggingIndicator(true);
    } else {
      robotContainer.setLoggingIndicator(false);
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    robotContainer.onAutoInit();

    autonomousCommand = robotContainer.getAutonomousCommand();
    isLogging.set(true);

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    robotContainer.onTeleopInit();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /** Initiates logging of active command names. */
  private void startCommandLogging() {
    final var counts = new HashMap<String, Integer>();
    BiConsumer<Command, Boolean> logger =
        (Command command, Boolean active) -> {
          final var name = command.getName();
          final var code = Integer.toHexString(command.hashCode());
          final var count = counts.getOrDefault(name, 0) + (active ? 1 : -1);
          counts.put(name, count);
          Logger.recordOutput("Commands/" + name + "_" + code, active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };

    final var scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize((Command command) -> logger.accept(command, true));
    scheduler.onCommandFinish((Command command) -> logger.accept(command, false));
    scheduler.onCommandInterrupt((Command command) -> logger.accept(command, false));
  }
}
