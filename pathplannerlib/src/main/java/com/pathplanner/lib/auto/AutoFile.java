package com.pathplanner.lib.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Filesystem;

/**
 * The contents of a PathPlanner auto file, independent of the command framework being used.
 *
 * @param command The command that the auto runs
 * @param resetOdom Should the auto reset odometry to the starting pose of the first path
 * @param choreoAuto True if the paths in this auto are Choreo trajectories
 */
public record AutoFile(CommandSpec command, boolean resetOdom, boolean choreoAuto) {
  /**
   * Load an auto file from the "deploy/pathplanner/autos" directory
   *
   * @param autoName The name of the auto to load
   * @return The loaded auto file
   * @throws IOException if the file cannot be read
   * @throws ParseException If the JSON cannot be parsed
   * @throws FileVersionException If the file version does not match the expected version
   */
  public static AutoFile fromFile(String autoName) throws IOException, ParseException {
    JSONObject json = readAutoJson(autoName);

    String version = json.get("version").toString();
    String[] versions = version.split("\\.");

    if (!versions[0].equals("2025")) {
      throw new FileVersionException(version, "2025.X", autoName + ".auto");
    }

    return fromJson(json);
  }

  /**
   * Create an auto file from json
   *
   * @param autoJson {@link org.json.simple.JSONObject} representing an auto
   * @return The auto file defined by the given json
   */
  public static AutoFile fromJson(JSONObject autoJson) {
    boolean choreoAuto = autoJson.get("choreoAuto") != null && (boolean) autoJson.get("choreoAuto");
    boolean resetOdom = autoJson.get("resetOdom") != null && (boolean) autoJson.get("resetOdom");
    CommandSpec command = CommandSpec.fromJson((JSONObject) autoJson.get("command"), choreoAuto);
    return new AutoFile(command, resetOdom, choreoAuto);
  }

  /**
   * Load all of the paths used by this auto, in the order they appear
   *
   * @return List of paths in the auto
   * @throws IOException if a path file cannot be read
   * @throws ParseException If a path's JSON cannot be parsed
   */
  public List<PathPlannerPath> loadPaths() throws IOException, ParseException {
    List<PathPlannerPath> paths = new ArrayList<>();
    for (CommandSpec.FollowPath pathCommand : command.getPathCommands()) {
      paths.add(pathCommand.loadPath(false));
    }
    return paths;
  }

  /**
   * Get the starting pose of this auto, which is the starting pose of its first path
   *
   * @param holonomic True if the robot has a holonomic drive train
   * @param mirror Should the paths be mirrored to the other side of the field
   * @return The starting pose, or null if this auto does not contain any paths
   * @throws IOException if a path file cannot be read
   * @throws ParseException If a path's JSON cannot be parsed
   */
  public Pose2d getStartingPose(boolean holonomic, boolean mirror)
      throws IOException, ParseException {
    List<CommandSpec.FollowPath> pathCommands = command.getPathCommands();
    if (pathCommands.isEmpty()) {
      return null;
    }

    PathPlannerPath path0 = pathCommands.get(0).loadPath(mirror);
    if (holonomic) {
      return new Pose2d(path0.getPoint(0).position, path0.getIdealStartingState().rotation());
    } else {
      return path0.getStartingDifferentialPose();
    }
  }

  /**
   * Get the names of all autos in the "deploy/pathplanner/autos" directory
   *
   * @return List of all auto names
   */
  public static List<String> getAllAutoNames() {
    File[] autoFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/autos").listFiles();

    if (autoFiles == null) {
      return new ArrayList<>();
    }

    return Stream.of(autoFiles)
        .filter(file -> !file.isDirectory())
        .map(File::getName)
        .filter(name -> name.endsWith(".auto"))
        .map(name -> name.substring(0, name.lastIndexOf(".")))
        .collect(Collectors.toList());
  }

  private static JSONObject readAutoJson(String autoName) throws IOException, ParseException {
    try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto")))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      return (JSONObject) new JSONParser().parse(fileContentBuilder.toString());
    }
  }
}
