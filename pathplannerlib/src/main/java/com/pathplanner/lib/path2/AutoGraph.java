package com.pathplanner.lib.path2;

import com.pathplanner.lib.util.FileVersionException;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Filesystem;

/**
 * An auto drawn in the PathPlanner 2027 app, loaded from a version 2027.1 .auto file.
 *
 * <p>An auto is a directed graph of steps. A step either follows a path or runs a command
 * registered with NamedCommands. The auto starts at the single step with no incoming branch. When a
 * step finishes, the auto moves on along its "finished" branch. A "condition" branch is taken as
 * soon as its named condition is true, canceling the step if it is still running.
 *
 * @param name The name of the auto
 * @param nodes The steps of the auto, in file order
 * @param branches The branches of the auto, in file order
 * @param startingPose The pose the robot starts at, relative to the blue alliance origin. The app
 *     saves it from the center of the field, and it is converted on load with {@link
 *     AppCoordinates}.
 * @param startingPoseInitialized True if a starting pose has been set in the app. The app sets it
 *     from the first path as soon as one is added.
 */
public record AutoGraph(
    String name,
    List<Node> nodes,
    List<Branch> branches,
    Pose2d startingPose,
    boolean startingPoseInitialized) {
  /**
   * Create an auto graph
   *
   * @param name The name of the auto
   * @param nodes The steps of the auto, in file order
   * @param branches The branches of the auto, in file order
   * @param startingPose The pose the robot starts at
   * @param startingPoseInitialized True if a starting pose has been set in the app
   */
  public AutoGraph {
    nodes = List.copyOf(nodes);
    branches = List.copyOf(branches);
    Set<String> ids = new HashSet<>();
    for (Node node : nodes) {
      if (!ids.add(node.id())) {
        throw new IllegalArgumentException("Duplicate graph ID: " + node.id());
      }
    }
    Set<String> targets = new HashSet<>();
    for (Branch branch : branches) {
      if (!ids.contains(branch.sourceId()) || !ids.contains(branch.targetId())) {
        throw new IllegalArgumentException("Branch " + branch.id() + " references a missing node");
      }
      targets.add(branch.targetId());
    }
    if (!nodes.isEmpty() && nodes.stream().filter(n -> !targets.contains(n.id())).count() != 1) {
      throw new IllegalArgumentException("Auto must have exactly one start node");
    }
  }

  /**
   * Load an auto from a 2027.1 auto file in the "deploy/pathplanner/autos" directory
   *
   * @param autoName The name of the auto to load
   * @return The auto graph
   * @throws IOException if the file cannot be read
   * @throws ParseException if the JSON cannot be parsed
   * @throws FileVersionException if the file is not a 2027.1 auto
   * @throws IllegalStateException if FlippingUtil's field size disagrees with the project's
   *     navgrid.json. See {@link AppCoordinates}.
   */
  public static AutoGraph fromFile(String autoName) throws IOException, ParseException {
    JSONObject json = readJson(autoName);
    String version = String.valueOf(json.get("version"));
    if (!FileVersion.isGraphFormat(version)) {
      throw new FileVersionException(
          version, FileVersion.GRAPH_FORMAT_VERSION + " or newer", autoName + ".auto");
    }
    AppCoordinates.checkFieldSize();
    return fromJson(autoName, json);
  }

  /**
   * Check if an auto file uses the 2027 graph format
   *
   * @param autoName The name of the auto in the "deploy/pathplanner/autos" directory
   * @return True if the file version is 2027.1 or newer
   * @throws IOException if the file cannot be read
   * @throws ParseException if the JSON cannot be parsed
   */
  public static boolean isGraphFile(String autoName) throws IOException, ParseException {
    return FileVersion.isGraphFormat(String.valueOf(readJson(autoName).get("version")));
  }

  /**
   * Create an auto graph from the JSON of a 2027.1 auto file
   *
   * @param name The name of the auto
   * @param json The JSON object of the auto file
   * @return The auto graph
   */
  public static AutoGraph fromJson(String name, JSONObject json) {
    List<Node> nodes = new ArrayList<>();
    for (Object nodeJson : (JSONArray) json.get("nodes")) {
      nodes.add(Node.fromJson((JSONObject) nodeJson));
    }
    List<Branch> branches = new ArrayList<>();
    for (Object branchJson : (JSONArray) json.get("branches")) {
      branches.add(Branch.fromJson((JSONObject) branchJson));
    }

    JSONObject poseJson = (JSONObject) json.get("startingPose");
    // The starting pose is saved from the center of the field, with its rotation in radians
    Pose2d startingPose =
        AppCoordinates.toBlueOrigin(
            new Pose2d(
                PathGraph.translation((JSONObject) poseJson.get("position")),
                Rotation2d.fromRadians(((Number) poseJson.get("rotation")).doubleValue())));
    boolean initialized =
        json.get("startingPoseInitialized") != null
            && (boolean) json.get("startingPoseInitialized");

    return new AutoGraph(name, nodes, branches, startingPose, initialized);
  }

  /**
   * Get the step the auto starts at
   *
   * @return The start step, or null if the auto has no steps
   */
  public Node getStartNode() {
    Set<String> targets = new HashSet<>();
    for (Branch branch : branches) {
      targets.add(branch.targetId());
    }
    return nodes.stream().filter(n -> !targets.contains(n.id())).findFirst().orElse(null);
  }

  /**
   * Get a step by its ID
   *
   * @param id The ID of the step
   * @return The step, or null if no step has that ID
   */
  public Node getNode(String id) {
    return nodes.stream().filter(n -> n.id().equals(id)).findFirst().orElse(null);
  }

  /**
   * Get the branches leaving a step, in file order
   *
   * @param nodeId The ID of the source step
   * @return The outgoing branches
   */
  public List<Branch> getOutgoingBranches(String nodeId) {
    return branches.stream().filter(branch -> branch.sourceId().equals(nodeId)).toList();
  }

  /**
   * Get the names of the paths this auto follows, without duplicates, in file order
   *
   * @return The path names
   */
  public List<String> getPathNames() {
    Map<String, Boolean> names = new LinkedHashMap<>();
    for (Node node : nodes) {
      if (node.pathName() != null && !node.pathName().isBlank()) {
        names.put(node.pathName(), true);
      }
    }
    return List.copyOf(names.keySet());
  }

  private static JSONObject readJson(String autoName) throws IOException, ParseException {
    File file =
        new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto");
    try (BufferedReader br = new BufferedReader(new FileReader(file))) {
      return (JSONObject) new JSONParser().parse(br);
    }
  }

  /**
   * A step of an auto. Exactly one of {@code pathName} and {@code commandName} is set, unless a
   * path step has no path selected in the app, in which case both are null.
   *
   * @param id The unique ID of the step
   * @param pathName The name of the path this step follows, for a path step
   * @param commandName The name of the NamedCommands command this step runs, for a command step
   * @param events Names of the events signaled when this step starts
   */
  public record Node(String id, String pathName, String commandName, List<String> events) {
    /**
     * Create a step
     *
     * @param id The unique ID of the step
     * @param pathName The name of the path this step follows, for a path step
     * @param commandName The name of the command this step runs, for a command step
     * @param events Names of the events signaled when this step starts
     */
    public Node {
      events = List.copyOf(events);
    }

    /**
     * Is this a path step
     *
     * @return True if this step follows a path
     */
    public boolean isPath() {
      return commandName == null;
    }

    static Node fromJson(JSONObject json) {
      String type = (String) json.get("type");
      List<String> events = strings((JSONArray) json.get("events"));
      return switch (type) {
        case "path" ->
            new Node((String) json.get("id"), (String) json.get("pathName"), null, events);
        case "external" ->
            new Node((String) json.get("id"), null, (String) json.get("commandName"), events);
        default -> throw new IllegalArgumentException("Unknown auto node type: " + type);
      };
    }
  }

  /**
   * A branch of an auto, from one step to another
   *
   * @param id The unique ID of the branch
   * @param sourceId The ID of the step this branch leaves
   * @param targetId The ID of the step this branch leads to
   * @param conditionName The name of the condition for a condition branch, or null for a "finished"
   *     branch
   * @param isCondition True for a condition branch, false for a "finished" branch
   * @param events Names of the events signaled when this branch is taken
   */
  public record Branch(
      String id,
      String sourceId,
      String targetId,
      String conditionName,
      boolean isCondition,
      List<String> events) {
    /**
     * Create a branch
     *
     * @param id The unique ID of the branch
     * @param sourceId The ID of the step this branch leaves
     * @param targetId The ID of the step this branch leads to
     * @param conditionName The name of the condition for a condition branch
     * @param isCondition True for a condition branch, false for a "finished" branch
     * @param events Names of the events signaled when this branch is taken
     */
    public Branch {
      events = List.copyOf(events);
    }

    static Branch fromJson(JSONObject json) {
      JSONObject transition = (JSONObject) json.get("transition");
      String type = (String) transition.get("type");
      boolean isCondition =
          switch (type) {
            case "finished" -> false;
            case "condition" -> true;
            default -> throw new IllegalArgumentException("Unknown auto transition type: " + type);
          };
      return new Branch(
          (String) json.get("id"),
          (String) json.get("sourceId"),
          (String) json.get("targetId"),
          isCondition ? (String) transition.get("conditionName") : null,
          isCondition,
          strings((JSONArray) json.get("events")));
    }
  }

  private static List<String> strings(JSONArray array) {
    List<String> strings = new ArrayList<>();
    if (array != null) {
      for (Object value : array) {
        strings.add((String) value);
      }
    }
    return strings;
  }
}
