package com.pathplanner.lib.path2;

/**
 * Version checks shared by the path and auto loaders.
 *
 * <p>PathPlanner 2027 writes graph-shaped path and auto files at version {@code 2027.1}. The app
 * accepts any version at or above that, so a file written by a newer compatible release still
 * loads. PathPlanner 2025 and 2026 wrote version {@code 2025.X}.
 */
public final class FileVersion {
  /** The first file version that uses the graph format */
  public static final String GRAPH_FORMAT_VERSION = "2027.1";

  private FileVersion() {}

  /**
   * Check if a file version uses the 2027 graph format
   *
   * @param version The "version" string from a path or auto file
   * @return True if the version is 2027.1 or newer
   */
  public static boolean isGraphFormat(String version) {
    return compare(version, GRAPH_FORMAT_VERSION) >= 0;
  }

  /**
   * Check if a file version uses the 2025 format, read by {@link
   * com.pathplanner.lib.path.PathPlannerPath}
   *
   * @param version The "version" string from a path or auto file
   * @return True if the major version is 2025
   */
  public static boolean isLegacyFormat(String version) {
    return version.split("\\.")[0].equals("2025");
  }

  private static int compare(String a, String b) {
    int[] partsA = parse(a);
    int[] partsB = parse(b);
    for (int i = 0; i < Math.max(partsA.length, partsB.length); i++) {
      int partA = i < partsA.length ? partsA[i] : 0;
      int partB = i < partsB.length ? partsB[i] : 0;
      if (partA != partB) {
        return Integer.compare(partA, partB);
      }
    }
    return 0;
  }

  private static int[] parse(String version) {
    String[] parts = version.split("\\.");
    int[] numbers = new int[parts.length];
    for (int i = 0; i < parts.length; i++) {
      try {
        numbers[i] = Integer.parseInt(parts[i]);
      } catch (NumberFormatException e) {
        return new int[] {-1};
      }
    }
    return numbers;
  }
}
