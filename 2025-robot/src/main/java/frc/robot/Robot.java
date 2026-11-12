// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;

@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  private final GCStatsCollector gcStatsCollector = new GCStatsCollector();

  private Command autonomousCommand;

  @Logged(name = "Container")
  private final RobotContainer robotContainer;

  @Logged(name = "State")
  private final RobotState robotState;

  private final Vision vision;

  public Robot() {
    robotState = RobotState.getInstance();
    vision = Vision.getInstance();
    robotContainer = new RobotContainer();

    if (Constants.Settings.rioLoggingEnabled) {
      DataLogManager.start(); // required for Epilogue data to be logged to disk

      DogLog.setOptions(
          new DogLogOptions()
              .withCaptureConsole(true)
              .withLogExtras(true)
              .withCaptureDs(true) // capture DS inputs
              .withCaptureNt(true) // required for Epilogue data to be logged to disk
          );

      // log metadata
      DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
      DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
      DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
      DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
      DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);

      if (Constants.Settings.epilogueLoggingEnabled) {
        // https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html
        Epilogue.configure(
            config -> {
              config.root = "/Robot";

              if (isSimulation()) {
                // if running in simulation, then we'd want to re-throw any errors that
                // occur so we can debug and fix them!
                config.errorHandler = ErrorHandler.crashOnError();
              }

              // change to CRITICAL for more detailed logging (where supported)
              config.minimumImportance = Logged.Importance.DEBUG;
            });

        Epilogue.bind(this);
      }
    } else {
      DogLog.setEnabled(false);
    }
  }

  @Override
  public void robotInit() {
    robotContainer.initValidation();
  }

  @Override
  public void robotPeriodic() {
    // uncomment to run periodic at real-time priority, may introduce unintended side-effects, see:
    // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/#real-time-thread-priority
    // Threads.setCurrentThreadPriority(true, 99);

    DogLog.timestamp("Periodic");

    Tunable.periodic();
    PhoenixUtil.refreshSignals();

    robotState.enterPeriodic();
    CommandScheduler.getInstance().run();
    robotState.exitPeriodic();

    // Threads.setCurrentThreadPriority(false, 10);

    if (Constants.Settings.statsEnabled) {
      DogLog.log("Stats/PeriodicMS", (RobotController.getFPGATime() - getLoopStartTime()) / 1000.0);
      gcStatsCollector.update();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    vision.enableLogging();
    RobotState.getInstance().enterAutonomous();

    autonomousCommand = robotContainer.getAutonomousCommand();

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
    RobotState.getInstance().enterTeleop();
    robotContainer.enableIntake();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    vision.disableLogging();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  /**
   * GC stats collector borrowed from AdvantageKit.
   * https://github.com/Mechanical-Advantage/AdvantageKit/blob/master/akit/src/main/java/org/littletonrobotics/junction/LoggedRobot.java
   */
  private static final class GCStatsCollector {
    private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] lastTimeMS = new long[gcBeans.size()];
    private final long[] lastCounts = new long[gcBeans.size()];

    public void update() {
      long timeMS = 0;
      long counts = 0;

      for (int i = 0; i < gcBeans.size(); i++) {
        final var gcBean = gcBeans.get(i);
        final var gcTimeMS = gcBean.getCollectionTime();
        final var gcCounts = gcBean.getCollectionCount();

        timeMS += gcTimeMS - lastTimeMS[i];
        counts += gcCounts - lastCounts[i];

        lastTimeMS[i] = gcTimeMS;
        lastCounts[i] = gcCounts;
      }

      DogLog.log("Stats/GCTimeMS", (double) timeMS);
      DogLog.log("Stats/GCCounts", (double) counts);
    }
  }
}
