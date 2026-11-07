// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootEpilogueBackend;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.CANduit;
import frc.robot.lib.EnergyTracker;
import frc.robot.lib.Hack;
import frc.robot.lib.MatchTime;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;

public class Robot extends TimedRobot {
  private final GCStatsCollector gcStatsCollector = new GCStatsCollector();
  private final Vision vision;
  private final CANduit canduit = new CANduit(Constants.CanID.CANDUIT);

  // spotless:off
  private final NetworkTable tableStats = NetworkTableInstance.getDefault().getTable("Stats");
  private final DoublePublisher statsTunableTopic = tableStats.getDoubleTopic("TunableMS").publish();
  private final DoublePublisher statsSignalsTopic = tableStats.getDoubleTopic("SignalsMS").publish();
  private final DoublePublisher statsSchedulerTopic = tableStats.getDoubleTopic("SchedulerMS").publish();
  private final DoublePublisher statsPeriodicTopic = tableStats.getDoubleTopic("PeriodicMS").publish();

  private final NetworkTable tableMeta = NetworkTableInstance.getDefault().getTable("Metadata");
  private final StringPublisher metaProjectNameTopic = tableMeta.getStringTopic("ProjectName").publish();
  private final StringPublisher metaBuildDateTopic = tableMeta.getStringTopic("BuildDate").publish();
  private final StringPublisher metaGitSHATopic = tableMeta.getStringTopic("GitSHA").publish();
  private final StringPublisher metaGitDateTopic = tableMeta.getStringTopic("GitDate").publish();
  private final StringPublisher metaGitBranchTopic = tableMeta.getStringTopic("GitBranch").publish();
  // spotless:on

  private Command autonomousCommand;

  private final EnergyTracker energyTracker;

  @Logged(name = "Container", importance = Importance.CRITICAL)
  private final RobotContainer robotContainer;

  @Logged(name = "State", importance = Importance.CRITICAL)
  private final RobotState robotState;

  @Logged(name = "MatchTime", importance = Importance.CRITICAL)
  private final MatchTime matchTime = new MatchTime(2026);

  public Robot() {
    vision = Vision.getInstance();
    energyTracker = EnergyTracker.getInstance();
    robotState = RobotState.getInstance();
    robotContainer = new RobotContainer(canduit);

    if (Constants.Settings.rioLoggingEnabled) {
      DataLogManager.start(); // required for Epilogue data to be logged to disk
      DriverStation.startDataLog(
          DataLogManager.getLog()); // capture both DS control and joystick data

      // log metadata
      metaProjectNameTopic.set(BuildConstants.MAVEN_NAME);
      metaBuildDateTopic.set(BuildConstants.BUILD_DATE);
      metaGitSHATopic.set(BuildConstants.GIT_SHA);
      metaGitDateTopic.set(BuildConstants.GIT_DATE);
      metaGitBranchTopic.set(BuildConstants.GIT_BRANCH);

      if (Constants.Settings.epilogueLoggingEnabled) {
        // https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html
        Epilogue.configure(
            config -> {
              config.root = "/Robot";

              // log to both the Phoenix 6 SignalLogger and NT4 backends
              config.backend =
                  EpilogueBackend.multi(
                      new HootEpilogueBackend(),
                      new NTEpilogueBackend(NetworkTableInstance.getDefault()));

              if (isSimulation()) {
                // if running in simulation, then we'd want to re-throw any errors that
                // occur so we can debug and fix them!
                config.errorHandler = ErrorHandler.crashOnError();
              }

              config.minimumImportance =
                  Constants.DeploymentMode == Constants.Deployment.kCompetition
                      ? Logged.Importance.INFO
                      : Logged.Importance.DEBUG;
            });

        Epilogue.bind(this);
      }
    }

    Hack.adjustLoopOverrun(this);
    Hack.silenceWarnings();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    matchTime.update(MatchTime.kGameData2026.get());

    // uncomment to run periodic at real-time priority, may introduce unintended side-effects, see:
    // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template/#real-time-thread-priority
    // Threads.setCurrentThreadPriority(true, 99);

    if (Constants.Settings.statsEnabled) {
      var before = RobotController.getTime();
      Tunable.periodic();
      var after = RobotController.getTime();
      statsTunableTopic.set((after - before) / 1000.0);

      before = after;
      PhoenixUtil.refreshSignals();
      after = RobotController.getTime();
      statsSignalsTopic.set((after - before) / 1000.0);

      before = after;
      canduit.update();
      robotContainer.periodic();
      energyTracker.periodic();
      robotState.enterPeriodic();
      CommandScheduler.getInstance().run();
      robotState.exitPeriodic();
      after = RobotController.getTime();
      statsSchedulerTopic.set((after - before) / 1000.0);
      statsPeriodicTopic.set((RobotController.getTime() - getLoopStartTime()) / 1000.0);
      gcStatsCollector.update();
    } else {
      Tunable.periodic();
      PhoenixUtil.refreshSignals();
      canduit.update();
      robotContainer.periodic();
      energyTracker.periodic();
      robotState.enterPeriodic();
      CommandScheduler.getInstance().run();
      robotState.exitPeriodic();
    }

    // Threads.setCurrentThreadPriority(false, 10);
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
    robotState.enterAutonomous();

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    robotState.enterTeleop();

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

    // spotless:off
    private final NetworkTable tableStats = NetworkTableInstance.getDefault().getTable("Stats");
    private final DoublePublisher gcTimeMSTopic = tableStats.getDoubleTopic("GCTimeMS").publish();
    private final DoublePublisher gcCountsTopic = tableStats.getDoubleTopic("GCCounts").publish();
    // spotless:on

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

      gcTimeMSTopic.set(timeMS);
      gcCountsTopic.set(counts);
    }
  }
}
