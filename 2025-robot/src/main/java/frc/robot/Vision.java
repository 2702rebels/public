package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null) instance = new Vision();
    return instance;
  }

  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable visionTable = nt.getTable("Vision");

  private final BooleanPublisher loggingPublisher =
      visionTable.getBooleanTopic("IsLogging").publish();

  private final BooleanSubscriber objectFlushing =
      visionTable.getBooleanTopic("Object/IsFlushing").subscribe(false);

  private final BooleanSubscriber localizationFlushing =
      visionTable.getBooleanTopic("Localization/IsFlushing").subscribe(false);

  private Vision() {
    loggingPublisher.setDefault(false);
  }

  /** Determines whether cameras are flushing logs to the media. */
  public boolean isCameraFlushing() {
    return objectFlushing.get() || localizationFlushing.get();
  }

  /** Enables camera real-time logging. */
  public void enableLogging() {
    loggingPublisher.set(true);
  }

  /** Disables camera real-time logging. */
  public void disableLogging() {
    loggingPublisher.set(false);
  }
}
