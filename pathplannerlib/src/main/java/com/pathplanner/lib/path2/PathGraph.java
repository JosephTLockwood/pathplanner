package com.pathplanner.lib.path2;

import com.pathplanner.lib.util.FileVersionException;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Filesystem;

/**
 * A path drawn in the PathPlanner 2027 app, loaded from a version 2027.1 .path file.
 *
 * <p>A path is a directed graph. Each node holds one waypoint, and each branch connects two nodes.
 * The robot starts at the single node with no incoming branch, and drives from node to node until
 * it reaches a node with no outgoing branch. Each branch says when the robot moves on to the next
 * waypoint: either a fixed distance before reaching it, or when a named condition becomes true.
 *
 * <p>Positions and headings are measured from WPILib's blue alliance origin, like every other pose
 * in PathPlannerLib. The app saves them measured from the center of the field, and they are
 * converted when the file is loaded. See {@link AppCoordinates}.
 */
public class PathGraph {
  private static final Map<String, PathGraph> cache = new HashMap<>();

  /** The name of the path, or an empty string if the path was not loaded from a file */
  public final String name;

  private final Map<String, Node> nodes;
  private final List<Branch> branches;
  private final Node root;

  /**
   * Create a path graph
   *
   * @param name The name of the path
   * @param nodes The nodes of the path, in file order
   * @param branches The branches of the path, in file order
   * @throws IllegalArgumentException if the graph is not a single-start acyclic graph
   */
  public PathGraph(String name, List<Node> nodes, List<Branch> branches) {
    this.name = name;
    this.nodes = new LinkedHashMap<>();
    for (Node node : nodes) {
      if (this.nodes.put(node.id(), node) != null) {
        throw new IllegalArgumentException("Duplicate graph ID: " + node.id());
      }
    }
    if (this.nodes.isEmpty()) {
      throw new IllegalArgumentException("A path must have at least one node");
    }
    this.branches = List.copyOf(branches);

    Set<String> targets = new HashSet<>();
    for (Branch branch : this.branches) {
      if (!this.nodes.containsKey(branch.sourceId())
          || !this.nodes.containsKey(branch.targetId())) {
        throw new IllegalArgumentException("Branch " + branch.id() + " references a missing node");
      }
      if (branch.sourceId().equals(branch.targetId())) {
        throw new IllegalArgumentException("Branch " + branch.id() + " is a self-link");
      }
      targets.add(branch.targetId());
    }

    List<Node> roots =
        this.nodes.values().stream().filter(node -> !targets.contains(node.id())).toList();
    if (roots.size() != 1) {
      throw new IllegalArgumentException(
          "Path must have exactly one start node; found " + roots.size());
    }
    this.root = roots.get(0);

    if (hasCycle()) {
      throw new IllegalArgumentException("Path graph contains a cycle");
    }
  }

  /**
   * Load a path from a 2027.1 path file in the "deploy/pathplanner/paths" directory
   *
   * @param pathName The name of the path to load
   * @return PathGraph created from the given file name
   * @throws IOException if the file cannot be read
   * @throws ParseException if the JSON cannot be parsed
   * @throws FileVersionException if the file is not a 2027.1 path
   * @throws IllegalStateException if FlippingUtil's field size disagrees with the project's
   *     navgrid.json. See {@link AppCoordinates}.
   */
  public static PathGraph fromPathFile(String pathName) throws IOException, ParseException {
    if (cache.containsKey(pathName)) {
      return cache.get(pathName);
    }

    JSONObject json = readJson(pathFile(pathName));
    String version = String.valueOf(json.get("version"));
    if (!FileVersion.isGraphFormat(version)) {
      throw new FileVersionException(
          version, FileVersion.GRAPH_FORMAT_VERSION + " or newer", pathName + ".path");
    }

    AppCoordinates.checkFieldSize();
    PathGraph path = fromJson(pathName, json);
    cache.put(pathName, path);
    return path;
  }

  /**
   * Check if a path file uses the 2027 graph format
   *
   * @param pathName The name of the path in the "deploy/pathplanner/paths" directory
   * @return True if the file version is 2027.1 or newer
   * @throws IOException if the file cannot be read
   * @throws ParseException if the JSON cannot be parsed
   */
  public static boolean isGraphFile(String pathName) throws IOException, ParseException {
    return FileVersion.isGraphFormat(String.valueOf(readJson(pathFile(pathName)).get("version")));
  }

  /** Clear the cache of loaded paths, so they are read from their files again */
  public static void clearCache() {
    cache.clear();
  }

  /**
   * Create a path graph from the JSON of a 2027.1 path file. Poses are converted from the app's
   * coordinates to the blue alliance origin with {@link AppCoordinates}, using the current
   * FlippingUtil field size.
   *
   * @param name The name of the path
   * @param json The JSON object of the path file
   * @return The path graph
   */
  public static PathGraph fromJson(String name, JSONObject json) {
    List<Node> nodes = new ArrayList<>();
    for (Object nodeJson : (JSONArray) json.get("nodes")) {
      nodes.add(Node.fromJson((JSONObject) nodeJson));
    }
    List<Branch> branches = new ArrayList<>();
    for (Object branchJson : (JSONArray) json.get("branches")) {
      branches.add(Branch.fromJson((JSONObject) branchJson));
    }
    return new PathGraph(name, nodes, branches);
  }

  /**
   * Get the node the path starts at
   *
   * @return The start node
   */
  public Node getStartNode() {
    return root;
  }

  /**
   * Get a node by its ID
   *
   * @param id The ID of the node
   * @return The node, or null if no node has that ID
   */
  public Node getNode(String id) {
    return nodes.get(id);
  }

  /**
   * Get all nodes, in file order
   *
   * @return The nodes of this path
   */
  public List<Node> getNodes() {
    return List.copyOf(nodes.values());
  }

  /**
   * Get all branches, in file order
   *
   * @return The branches of this path
   */
  public List<Branch> getBranches() {
    return branches;
  }

  /**
   * Get the branches leaving a node, in file order
   *
   * @param nodeId The ID of the source node
   * @return The outgoing branches
   */
  public List<Branch> getOutgoingBranches(String nodeId) {
    return branches.stream().filter(branch -> branch.sourceId().equals(nodeId)).toList();
  }

  /**
   * Flip this path to the other alliance with {@link com.pathplanner.lib.util.FlippingUtil}
   *
   * @return The flipped path
   */
  public PathGraph flip() {
    return withWaypoints(node -> node.waypoint().flip());
  }

  /**
   * Mirror this path to the other side of the current alliance
   *
   * @return The mirrored path
   */
  public PathGraph mirror() {
    return withWaypoints(node -> node.waypoint().mirror());
  }

  private PathGraph withWaypoints(java.util.function.Function<Node, GraphWaypoint> transform) {
    List<Node> newNodes = new ArrayList<>();
    for (Node node : nodes.values()) {
      newNodes.add(
          new Node(
              node.id(),
              transform.apply(node),
              node.endToleranceMeters(),
              node.endToleranceRadians()));
    }
    return new PathGraph(name, newNodes, branches);
  }

  private boolean hasCycle() {
    Map<String, Integer> states = new HashMap<>();
    for (String id : nodes.keySet()) {
      if (visit(id, states)) {
        return true;
      }
    }
    return false;
  }

  private boolean visit(String id, Map<String, Integer> states) {
    int state = states.getOrDefault(id, 0);
    if (state == 1) {
      return true;
    }
    if (state == 2) {
      return false;
    }
    states.put(id, 1);
    for (Branch branch : getOutgoingBranches(id)) {
      if (visit(branch.targetId(), states)) {
        return true;
      }
    }
    states.put(id, 2);
    return false;
  }

  private static File pathFile(String pathName) {
    return new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path");
  }

  private static JSONObject readJson(File file) throws IOException, ParseException {
    try (BufferedReader br = new BufferedReader(new FileReader(file))) {
      return (JSONObject) new JSONParser().parse(br);
    }
  }

  /**
   * A node of a path graph
   *
   * @param id The unique ID of the node
   * @param waypoint The waypoint at this node
   * @param endToleranceMeters Distance tolerance when the path ends at this node, in meters
   * @param endToleranceRadians Heading tolerance when the path ends at this node, in radians
   */
  public record Node(
      String id, GraphWaypoint waypoint, double endToleranceMeters, double endToleranceRadians) {
    /** Distance tolerance used when a file leaves it out, matching the app's default */
    public static final double DEFAULT_END_TOLERANCE_METERS = 0.1;

    /** Heading tolerance used when a file leaves it out, matching the app's default */
    public static final double DEFAULT_END_TOLERANCE_DEGREES = 2.0;

    static Node fromJson(JSONObject json) {
      JSONObject waypointJson = (JSONObject) json.get("waypoint");
      JSONObject toleranceJson = (JSONObject) json.get("endTolerance");
      return new Node(
          (String) json.get("id"),
          waypointFromJson(waypointJson),
          number(toleranceJson, "distanceMeters", DEFAULT_END_TOLERANCE_METERS),
          Math.toRadians(number(toleranceJson, "angleDegrees", DEFAULT_END_TOLERANCE_DEGREES)));
    }

    private static GraphWaypoint waypointFromJson(JSONObject json) {
      String type = (String) json.get("type");
      // The app saves poses from the center of the field. Convert them to the blue origin here,
      // once, so nothing after loading sees the app's coordinates. A rotation offset is relative to
      // the direction of the target, so it does not change.
      Translation2d position =
          AppCoordinates.toBlueOrigin(translation((JSONObject) json.get("position")));
      Rotation2d rotation = null;
      Translation2d target = null;
      Rotation2d offset = Rotation2d.ZERO;
      boolean unprofiled = false;
      switch (type) {
        case "pose" -> {
          // Pose waypoint headings are saved in radians
          JSONObject rotationJson = (JSONObject) json.get("rotation");
          rotation =
              AppCoordinates.toBlueOrigin(
                  Rotation2d.fromRadians(((Number) rotationJson.get("value")).doubleValue()));
        }
        case "translation" -> {}
        case "pointTowards" -> {
          target =
              AppCoordinates.toBlueOrigin(translation((JSONObject) json.get("targetPosition")));
          // The point towards offset is saved in degrees
          offset = Rotation2d.fromDegrees(number(json, "rotationOffset", 0.0));
          unprofiled = json.get("unprofiled") != null && (boolean) json.get("unprofiled");
        }
        default -> throw new IllegalArgumentException("Unknown waypoint type: " + type);
      }

      List<String> events = new ArrayList<>();
      if (json.get("events") != null) {
        for (Object event : (JSONArray) json.get("events")) {
          events.add((String) event);
        }
      }

      // Angular limits are saved in degrees. The app writes the project defaults into a waypoint
      // that uses them, so the saved numbers are the ones to follow either way.
      return new GraphWaypoint(
          position,
          rotation,
          target,
          offset,
          unprofiled,
          number(json, "maxVelocity", GraphWaypoint.DEFAULT_MAX_VELOCITY_MPS),
          Math.toRadians(
              number(json, "maxAngularVelocity", GraphWaypoint.DEFAULT_MAX_ANGULAR_VELOCITY_DEG)),
          Math.toRadians(
              number(
                  json,
                  "maxAngularAcceleration",
                  GraphWaypoint.DEFAULT_MAX_ANGULAR_ACCELERATION_DEG)),
          events);
    }
  }

  /**
   * A branch of a path graph, from one node to another
   *
   * @param id The unique ID of the branch
   * @param sourceId The ID of the node this branch leaves
   * @param targetId The ID of the node this branch leads to
   * @param transition When the robot moves on along this branch
   * @param events Events placed along this branch
   */
  public record Branch(
      String id, String sourceId, String targetId, Transition transition, List<Event> events) {
    /**
     * Create a branch
     *
     * @param id The unique ID of the branch
     * @param sourceId The ID of the node this branch leaves
     * @param targetId The ID of the node this branch leads to
     * @param transition When the robot moves on along this branch
     * @param events Events placed along this branch
     */
    public Branch {
      events = List.copyOf(events);
    }

    static Branch fromJson(JSONObject json) {
      List<Event> events = new ArrayList<>();
      if (json.get("events") != null) {
        for (Object eventJson : (JSONArray) json.get("events")) {
          JSONObject event = (JSONObject) eventJson;
          events.add(
              new Event(
                  (String) event.get("name"), ((Number) event.get("position")).doubleValue()));
        }
      }
      return new Branch(
          (String) json.get("id"),
          (String) json.get("sourceId"),
          (String) json.get("targetId"),
          Transition.fromJson((JSONObject) json.get("transition")),
          events);
    }
  }

  /**
   * An event placed along a branch
   *
   * @param name The name of the event
   * @param position How far along the branch the event is, from 0.0 at the source waypoint to 1.0
   *     at the target waypoint
   */
  public record Event(String name, double position) {}

  /** When the robot moves on from one waypoint to the next */
  public sealed interface Transition {
    /**
     * The distance before the target waypoint at which the robot starts driving to the waypoint
     * after it. For a condition branch this is the distance the app previews.
     *
     * @return The hand-off distance, in meters
     */
    double handoffDistanceMeters();

    /**
     * Create a transition from the JSON of a branch
     *
     * @param json The "transition" object of a branch
     * @return The transition
     */
    static Transition fromJson(JSONObject json) {
      String type = (String) json.get("type");
      return switch (type) {
        case "distance" ->
            new DistanceTransition(((Number) json.get("distanceMeters")).doubleValue());
        case "condition" ->
            new ConditionTransition(
                (String) json.get("conditionName"),
                number(
                    json,
                    "previewDistanceMeters",
                    ConditionTransition.DEFAULT_PREVIEW_DISTANCE_METERS));
        default -> throw new IllegalArgumentException("Unknown path transition type: " + type);
      };
    }
  }

  /**
   * Move on to the next waypoint a fixed distance before reaching this one
   *
   * @param distanceMeters The hand-off distance, in meters
   */
  public record DistanceTransition(double distanceMeters) implements Transition {
    @Override
    public double handoffDistanceMeters() {
      return distanceMeters;
    }
  }

  /**
   * Take this branch when a named condition is true. See {@link
   * com.pathplanner.lib.auto.NamedConditions}.
   *
   * @param conditionName The name of the condition, or null if none was selected in the app
   * @param previewDistanceMeters The hand-off distance once this branch is taken, in meters. The
   *     app also uses it to preview the branch.
   */
  public record ConditionTransition(String conditionName, double previewDistanceMeters)
      implements Transition {
    /** Preview distance used when a file leaves it out, matching the app's default */
    public static final double DEFAULT_PREVIEW_DISTANCE_METERS = 0.25;

    @Override
    public double handoffDistanceMeters() {
      return previewDistanceMeters;
    }
  }

  static Translation2d translation(JSONObject json) {
    return new Translation2d(
        ((Number) json.get("x")).doubleValue(), ((Number) json.get("y")).doubleValue());
  }

  static double number(JSONObject json, String key, double defaultValue) {
    if (json == null || json.get(key) == null) {
      return defaultValue;
    }
    return ((Number) json.get(key)).doubleValue();
  }
}
