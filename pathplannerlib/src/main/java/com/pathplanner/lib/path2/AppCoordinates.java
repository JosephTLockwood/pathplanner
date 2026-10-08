package com.pathplanner.lib.path2;

import com.pathplanner.lib.util.FlippingUtil;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Filesystem;

/**
 * Converts poses saved by the PathPlanner 2027 app into WPILib's blue alliance origin, which is the
 * coordinate system every pose PathPlannerLib takes or returns is in.
 *
 * <p>The 2027 app saves 2027.1 files measured from the center of the field. It draws the official
 * field images turned half a turn, red alliance wall on the left, with +X to the right and +Y up.
 * So +X points towards the blue alliance wall, the opposite of WPILib, and a saved pose is turned
 * half a turn and shifted relative to the blue origin:
 *
 * <pre>
 *   blueX = fieldLength / 2 - x
 *   blueY = fieldWidth / 2 - y
 *   blueHeading = heading + 180 degrees
 * </pre>
 *
 * <p>The field length and width are {@link FlippingUtil#fieldSizeX} and {@link
 * FlippingUtil#fieldSizeY}, the same numbers used to flip paths for the red alliance. Their
 * defaults are the 2026 field. A wrong size shifts every path, so loading a 2027.1 file throws if
 * they disagree by more than {@value #FIELD_SIZE_TOLERANCE_METERS} m with the field size in the
 * project's deploy/pathplanner/navgrid.json, which the app writes from its Navigation Grid page.
 *
 * <p>Custom field images are not turned by the app, so files drawn on one are not in this
 * coordinate system. Only the official field images are supported.
 */
public final class AppCoordinates {
  /** How far the field size may differ from the navgrid's before loading fails, in meters */
  public static final double FIELD_SIZE_TOLERANCE_METERS = 0.02;

  private AppCoordinates() {}

  /**
   * Convert a position saved by the 2027 app to the blue alliance origin
   *
   * @param appPosition The position measured from the center of the field, as the app saves it
   * @return The position measured from the blue alliance origin
   */
  public static Translation2d toBlueOrigin(Translation2d appPosition) {
    return new Translation2d(
        FlippingUtil.fieldSizeX / 2.0 - appPosition.getX(),
        FlippingUtil.fieldSizeY / 2.0 - appPosition.getY());
  }

  /**
   * Convert a heading saved by the 2027 app to the blue alliance origin
   *
   * @param appHeading The heading as the app saves it
   * @return The heading in the blue alliance origin
   */
  public static Rotation2d toBlueOrigin(Rotation2d appHeading) {
    return appHeading.plus(Rotation2d.PI);
  }

  /**
   * Convert a pose saved by the 2027 app to the blue alliance origin
   *
   * @param appPose The pose measured from the center of the field, as the app saves it
   * @return The pose measured from the blue alliance origin
   */
  public static Pose2d toBlueOrigin(Pose2d appPose) {
    return new Pose2d(toBlueOrigin(appPose.getTranslation()), toBlueOrigin(appPose.getRotation()));
  }

  /**
   * Check that the field size used for the conversion matches the project's navgrid.json. Does
   * nothing if the project has no navgrid.json.
   *
   * @throws IllegalStateException if the sizes disagree
   */
  public static void checkFieldSize() {
    File navGrid = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
    if (!navGrid.exists()) {
      return;
    }

    JSONObject fieldSize;
    try (BufferedReader br = new BufferedReader(new FileReader(navGrid))) {
      fieldSize = (JSONObject) ((JSONObject) new JSONParser().parse(br)).get("field_size");
    } catch (IOException | ParseException e) {
      throw new IllegalStateException("Could not read pathplanner/navgrid.json", e);
    }
    if (fieldSize == null) {
      return;
    }

    double length = ((Number) fieldSize.get("x")).doubleValue();
    double width = ((Number) fieldSize.get("y")).doubleValue();
    if (Math.abs(length - FlippingUtil.fieldSizeX) > FIELD_SIZE_TOLERANCE_METERS
        || Math.abs(width - FlippingUtil.fieldSizeY) > FIELD_SIZE_TOLERANCE_METERS) {
      throw new IllegalStateException(
          String.format(
              "The field is %.3f x %.3f m in pathplanner/navgrid.json but %.3f x %.3f m in "
                  + "FlippingUtil.fieldSizeX/fieldSizeY. 2027 paths are converted with the "
                  + "FlippingUtil size, so every path would be shifted. Set FlippingUtil.fieldSizeX "
                  + "and fieldSizeY before loading paths, or fix the field size in the app's "
                  + "Navigation Grid page.",
              length, width, FlippingUtil.fieldSizeX, FlippingUtil.fieldSizeY));
    }
  }
}
