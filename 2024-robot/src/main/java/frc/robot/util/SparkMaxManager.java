// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.BuildConstants;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.function.BooleanSupplier;

/**
 * Facilitates initialization of SparkMax devices and determines whether to burn SparkMax configs to
 * flash.
 */
public class SparkMaxManager {
  public static final int configCANTimeout = 500;
  public static final int maxRetries = 10;

  private static final String buildDateFile = "/home/lvuser/build-date.txt";
  private static boolean shouldBurn = false;

  private SparkMaxManager() {}

  /** Returns 1 if the last operation resulted in error, 0 otherwise. */
  public static int check(REVLibError error) {
    return error == REVLibError.kOk ? 0 : 1;
  }

  /**
   * Runs initialization until it succeeds or maximum number of retries has been reached.
   *
   * @param init initialization function that must return true on success
   * @param maxRetries maximum number of retries
   */
  public static void initialize(BooleanSupplier init, int maxRetries) {
    int retries = 0;
    while (!init.getAsBoolean()) {
      retries++;
      if (retries > maxRetries) {
        System.out.println(
            "[SparkMaxManager] Initialization unsuccessful after " + retries + " attempts");
        break;
      }
    }
  }

  /**
   * Runs initialization on the specified SparkMax controller with factory reset and burn until
   * initilization succeeds or the maximum number of retries has been reached.
   *
   * @param sparkMax SparkMax controller
   * @param init initialization function that must return true on success
   */
  public static void initializeAndBurn(CANSparkMax sparkMax, BooleanSupplier init) {
    sparkMax.setCANTimeout(SparkMaxManager.configCANTimeout);

    // factory reset, so we get the SPARK MAX to a known state before configuring them;
    // this is useful in case a SPARK MAX is swapped out
    if (SparkMaxManager.shouldBurn()) {
      initialize(() -> check(sparkMax.restoreFactoryDefaults()) == 0, maxRetries);
    }

    initialize(init, maxRetries);

    // make SetParameter calls non-blocking
    sparkMax.setCANTimeout(0);

    if (SparkMaxManager.shouldBurn()) {
      sparkMax.burnFlash();
    }
  }

  public static boolean shouldBurn() {
    return shouldBurn;
  }

  public static void update() {
    if (RobotBase.isSimulation()) {
      shouldBurn = false;
      return;
    }

    File file = new File(buildDateFile);
    if (!file.exists()) {

      // No build date file, burn flash
      shouldBurn = true;
    } else {

      // Read previous build date
      String previousBuildDate = "";
      try {
        previousBuildDate =
            new String(Files.readAllBytes(Paths.get(buildDateFile)), StandardCharsets.UTF_8);
      } catch (IOException e) {
        e.printStackTrace();
      }

      shouldBurn = !previousBuildDate.equals(BuildConstants.BUILD_DATE);
    }

    try {
      FileWriter fileWriter = new FileWriter(buildDateFile);
      fileWriter.write(BuildConstants.BUILD_DATE);
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (shouldBurn) {
      System.out.println("[SparkMaxManager] Build date changed, burning SparkMax flash");
    } else {
      System.out.println("[SparkMaxManager] Build date unchanged, will not burn SparkMax flash");
    }
  }
}
