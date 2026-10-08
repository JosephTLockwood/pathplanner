package com.pathplanner.lib.path2;

import com.pathplanner.lib.path.PathPlannerPath;
import java.io.File;
import java.net.URISyntaxException;

/**
 * Points the deploy directory at a robot project saved by the PathPlanner app. WPILib's
 * Filesystem.getDeployDirectory() is "src/main/deploy" under the working directory in simulation,
 * so this changes the working directory property until {@link #close()}.
 *
 * <p>The files under src/test/resources/project2027 were saved by the PathPlanner app
 * v0.0.0-dev-12, built from new-path-2027-commands-v3 at 43f3aa9. The paths named "Start to Shoot"
 * and "Shoot to Neutral Zone" and the auto "Shoot and Leave" are 2025.0 files from Workshop-Code's
 * swerve-pathplanner branch, which that app does not open.
 */
public final class FixtureProject implements AutoCloseable {
  /** Time the app's preview showed for the "Straight" path, in seconds */
  public static final double STRAIGHT_PREVIEW_SECONDS = 1.36;

  /** Time the app's preview showed for the "Rotate Events" path, in seconds */
  public static final double ROTATE_EVENTS_PREVIEW_SECONDS = 2.80;

  /** Time the app's preview showed for the "Leave Start" path, in seconds */
  public static final double LEAVE_START_PREVIEW_SECONDS = 1.24;

  private final String previousUserDir;

  private FixtureProject(String resource) {
    previousUserDir = System.getProperty("user.dir");
    try {
      File root = new File(FixtureProject.class.getClassLoader().getResource(resource).toURI());
      System.setProperty("user.dir", root.getAbsolutePath());
    } catch (URISyntaxException e) {
      throw new IllegalStateException(e);
    }
    PathGraph.clearCache();
    PathPlannerPath.clearCache();
  }

  /**
   * Use the project the 2027 app saved
   *
   * @return The open project. Close it to restore the working directory.
   */
  public static FixtureProject open2027() {
    return new FixtureProject("project2027");
  }

  /**
   * Use a project whose settings.json was saved by the 2026 app
   *
   * @return The open project. Close it to restore the working directory.
   */
  public static FixtureProject open2025() {
    return new FixtureProject("project2025");
  }

  @Override
  public void close() {
    System.setProperty("user.dir", previousUserDir);
    PathGraph.clearCache();
    PathPlannerPath.clearCache();
  }
}
