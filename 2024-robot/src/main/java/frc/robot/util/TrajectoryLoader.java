package frc.robot.util;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public final class TrajectoryLoader {
  /** Loads choreo trajectories from the deployment folder. */
  public static Map<String, ChoreoTrajectory> loadChoreoTrajectories() {
    Set<String> names;
    try {
      if (Robot.isReal()) {
        names = enumerateFiles("/home/lvuser/deploy/choreo");
      } else {
        names = enumerateFiles("src/main/deploy/choreo");
      }
    } catch (IOException e) {
      DriverStation.reportError("Failed to enumerate Choreo trajectories", true);
      return null;
    }

    return names.stream()
        .collect(
            Collectors.toMap(
                entry -> entry.replace(".traj", ""),
                entry -> {
                  System.out.println("Loading trajectory: " + entry);
                  return Choreo.getTrajectory(entry.replace(".traj", ""));
                }));
  }

  private static Set<String> enumerateFiles(String folder) throws IOException {
    try (var stream = Files.list(Paths.get(folder))) {
      return stream
          .filter(
              file ->
                  !Files.isDirectory(file) && !file.getFileName().toString().endsWith(".1.traj"))
          .map(Path::getFileName)
          .map(Path::toString)
          .collect(Collectors.toSet());
    }
  }
}
