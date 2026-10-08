package com.pathplanner.lib.path2;

import com.pathplanner.lib.auto.NamedConditions;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventConditions;
import com.pathplanner.lib.follower.ActivePathState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Follows a 2027 {@link PathGraph}. This contains the path following logic shared by the commands
 * of each supported command framework, and does not depend on a command framework itself.
 *
 * <p>A path is followed by calling {@link #start()}, then calling {@link #follow()} every 20 ms
 * loop until {@link #isFinished()} returns true, then calling {@link #stop(boolean)}.
 *
 * <p>Each loop, a {@link GraphController} computes the speeds to drive at, and a {@link
 * SwerveSetpointGenerator} limits them to what the drivetrain configured in the app can do. These
 * are the same two steps the app uses to preview a path.
 *
 * <p>When the path branches, the branch to take is chosen while the robot drives to the branch's
 * source waypoint, and stays chosen once the robot moves past that waypoint. The first condition
 * branch whose {@link NamedConditions named condition} is true is taken, in the order the branches
 * are saved in the file. If no condition is true, the first distance branch is taken. If there is
 * no distance branch either, the robot drives to the waypoint and waits there until a condition
 * becomes true.
 *
 * <p>Events are signaled through {@link EventConditions}, which is what event triggers read. An
 * event on a branch is signaled when the robot is that fraction of the way along the branch,
 * measured as the app measures it. An event on a waypoint is signaled when the robot reaches the
 * waypoint: the start waypoint when the path starts, each later waypoint when the robot moves on
 * from it, and the end waypoint when the path finishes.
 */
public class GraphFollower {
  /** The max steering speed of a swerve module the app's preview uses, in radians per second */
  public static final double MAX_STEER_VELOCITY_RAD_PER_SEC = 10.0 * 2.0 * Math.PI;

  private static final double INPUT_VOLTAGE = 12.0;

  private final PathGraph originalPath;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisVelocities> speedsSupplier;
  private final BiConsumer<ChassisVelocities, DriveFeedforwards> output;
  private final RobotConfig robotConfig;
  private final BooleanSupplier shouldFlipPath;
  private final SwerveSetpointGenerator setpointGenerator;
  private final Consumer<String> eventHandler;

  private PathGraph path;
  private GraphController controller;
  private SwerveSetpoint previousSetpoint;
  private final List<PathGraph.Node> routeNodes = new ArrayList<>();
  private final List<PathGraph.Branch> routeBranches = new ArrayList<>();
  private final Map<String, BooleanSupplier> conditions = new HashMap<>();
  private final Set<String> firedBranchEvents = new HashSet<>();
  private int nodesReached = 0;
  private boolean following = false;

  /**
   * Construct a path graph follower
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot,
   *     relative to the blue alliance origin, as for 2025 paths
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each swerve module, in FL, FR, BL, BR order
   * @param robotConfig The robot configuration. This must be a holonomic drivetrain, the only kind
   *     the 2027 app supports.
   * @param shouldFlipPath Should the path be flipped to the other side of the field
   */
  public GraphFollower(
      PathGraph path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath) {
    this(
        path,
        poseSupplier,
        speedsSupplier,
        output,
        robotConfig,
        shouldFlipPath,
        EventConditions::pulseEvent);
  }

  /**
   * Construct a path graph follower that reports events to a custom handler
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field
   * @param eventHandler Called with the name of each event as it is reached
   */
  public GraphFollower(
      PathGraph path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Consumer<String> eventHandler) {
    if (!robotConfig.isHolonomic) {
      throw new IllegalArgumentException(
          "2027 paths can only be followed by a holonomic drivetrain");
    }
    this.originalPath = path;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = output;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;
    this.eventHandler = eventHandler;
    this.setpointGenerator =
        new SwerveSetpointGenerator(robotConfig, MAX_STEER_VELOCITY_RAD_PER_SEC);
    this.path = path;
  }

  /** Start following the path. This flips the path if needed. */
  public void start() {
    path = shouldFlipPath.getAsBoolean() ? originalPath.flip() : originalPath;

    conditions.clear();
    for (PathGraph.Branch branch : path.getBranches()) {
      if (branch.transition() instanceof PathGraph.ConditionTransition condition) {
        conditions.computeIfAbsent(
            String.valueOf(condition.conditionName()),
            name -> NamedConditions.getCondition(condition.conditionName()));
      }
    }

    routeNodes.clear();
    routeBranches.clear();
    routeNodes.add(path.getStartNode());
    extendRoute();

    Pose2d currentPose = poseSupplier.get();
    ChassisVelocities currentSpeeds = speedsSupplier.get();
    PathGraph.Node leaf = routeNodes.getLast();
    controller =
        new GraphController(
            targets(),
            leaf.endToleranceMeters(),
            leaf.endToleranceRadians(),
            currentPose,
            currentSpeeds,
            false);
    previousSetpoint =
        new SwerveSetpoint(
            currentSpeeds,
            robotConfig.toSwerveModuleStates(currentSpeeds),
            DriveFeedforwards.zeros(robotConfig.numModules));

    firedBranchEvents.clear();
    nodesReached = 0;
    reachNodesBefore(1);

    ActivePathState.setCurrentPathName(originalPath.name);
    PathPlannerLogging.logActivePathPoses(routePoses());
    following = true;
  }

  /** Drive the robot along the path. This should be called every 20 ms loop. */
  public void follow() {
    Pose2d currentPose = poseSupplier.get();
    ChassisVelocities currentSpeeds = speedsSupplier.get();

    int previousTarget = controller.getTargetWaypointIndex();
    chooseBranches(previousTarget);
    signalBranchEvents(previousTarget, currentPose);
    ChassisVelocities desiredSpeeds = controller.calculate(currentPose, currentSpeeds);
    int target = controller.getTargetWaypointIndex();
    if (target != previousTarget) {
      signalBranchEvents(target, currentPose);
      reachNodesBefore(target);
    }

    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, desiredSpeeds, null, GraphController.PERIOD_SECONDS, INPUT_VOLTAGE);

    PathPlannerLogging.logCurrentPose(currentPose);
    PathGraph.Node targetNode = routeNodes.get(target);
    Rotation2d targetHeading = targetNode.waypoint().rotation();
    PathPlannerLogging.logTargetPose(
        new Pose2d(
            targetNode.waypoint().position(),
            targetHeading != null ? targetHeading : currentPose.getRotation()));

    output.accept(previousSetpoint.robotRelativeSpeeds(), previousSetpoint.feedforwards());
  }

  /**
   * Check if the path has been completed. A path that is waiting at a waypoint for a condition is
   * not complete.
   *
   * @return True if the robot has reached the end waypoint within its tolerances
   */
  public boolean isFinished() {
    return controller.isFinished() && path.getOutgoingBranches(routeNodes.getLast().id()).isEmpty();
  }

  /**
   * Stop following the path. Does nothing if the path is not being followed.
   *
   * <p>A path that finishes sends zero speeds, so the robot stops at the end waypoint. A path that
   * is interrupted sends nothing, so the command that interrupted it is free to drive. The last
   * speeds sent stay in effect until something else drives the robot.
   *
   * @param interrupted True if the path was interrupted before it was finished
   */
  public void stop(boolean interrupted) {
    if (!following) {
      return;
    }
    following = false;

    if (!interrupted) {
      reachNodesBefore(routeNodes.size());
      output.accept(new ChassisVelocities(), DriveFeedforwards.zeros(robotConfig.numModules));
    }

    PathPlannerLogging.logActivePathPoses(new ArrayList<>());
  }

  /**
   * Get the waypoints of the route the robot is currently taking through the path
   *
   * @return The nodes of the current route, starting at the start node
   */
  public List<PathGraph.Node> getRoute() {
    return List.copyOf(routeNodes);
  }

  /**
   * Get the index in {@link #getRoute()} of the waypoint the robot is driving to
   *
   * @return The target waypoint index
   */
  public int getTargetWaypointIndex() {
    return controller.getTargetWaypointIndex();
  }

  /**
   * Re-choose the branch at each node the robot has not yet moved on from. A node's choice is
   * locked once the robot targets the node after it.
   */
  private void chooseBranches(int targetIndex) {
    int keepNodes = Math.max(targetIndex + 1, 1);
    List<PathGraph.Node> oldNodes = List.copyOf(routeNodes);
    List<PathGraph.Branch> oldBranches = List.copyOf(routeBranches);
    while (routeNodes.size() > keepNodes) {
      routeNodes.removeLast();
      routeBranches.removeLast();
    }
    extendRoute();
    if (!routeNodes.equals(oldNodes) || !routeBranches.equals(oldBranches)) {
      controller.setWaypoints(targets());
      PathPlannerLogging.logActivePathPoses(routePoses());
    }
  }

  private void extendRoute() {
    while (true) {
      PathGraph.Branch next = chooseBranch(routeNodes.getLast());
      if (next == null) {
        return;
      }
      routeBranches.add(next);
      routeNodes.add(path.getNode(next.targetId()));
    }
  }

  private PathGraph.Branch chooseBranch(PathGraph.Node node) {
    List<PathGraph.Branch> outgoing = path.getOutgoingBranches(node.id());
    for (PathGraph.Branch branch : outgoing) {
      if (branch.transition() instanceof PathGraph.ConditionTransition condition
          && conditions.get(String.valueOf(condition.conditionName())).getAsBoolean()) {
        return branch;
      }
    }
    for (PathGraph.Branch branch : outgoing) {
      if (branch.transition() instanceof PathGraph.DistanceTransition) {
        return branch;
      }
    }
    return null;
  }

  private List<GraphController.Target> targets() {
    List<GraphController.Target> targets = new ArrayList<>();
    for (int i = 0; i < routeNodes.size(); i++) {
      double handoff =
          i < routeBranches.size() ? routeBranches.get(i).transition().handoffDistanceMeters() : 0;
      targets.add(new GraphController.Target(routeNodes.get(i).waypoint(), handoff));
    }
    return targets;
  }

  /**
   * Signal the events on the branch that leads to the target waypoint, the way the app's preview
   * places them: an event fires once the robot's distance to the target, as a fraction of the
   * branch length, is at most one minus the event's position.
   */
  private void signalBranchEvents(int targetIndex, Pose2d currentPose) {
    int branchIndex = targetIndex - 1;
    if (branchIndex < 0 || branchIndex >= routeBranches.size()) {
      return;
    }
    PathGraph.Branch branch = routeBranches.get(branchIndex);
    Translation2d source = routeNodes.get(branchIndex).waypoint().position();
    Translation2d target = routeNodes.get(targetIndex).waypoint().position();
    double distance = source.getDistance(target);
    if (distance == 0) {
      return;
    }
    double ratio = currentPose.getTranslation().getDistance(target) / distance;
    for (int i = 0; i < branch.events().size(); i++) {
      PathGraph.Event event = branch.events().get(i);
      String key = branchIndex + ":" + i;
      if (!firedBranchEvents.contains(key) && ratio <= 1 - event.position()) {
        firedBranchEvents.add(key);
        eventHandler.accept(event.name());
      }
    }
  }

  private void reachNodesBefore(int index) {
    while (nodesReached < Math.min(index, routeNodes.size())) {
      for (String event : routeNodes.get(nodesReached).waypoint().events()) {
        eventHandler.accept(event);
      }
      nodesReached++;
    }
  }

  private List<Pose2d> routePoses() {
    List<Pose2d> poses = new ArrayList<>();
    for (PathGraph.Node node : routeNodes) {
      Rotation2d rotation = node.waypoint().rotation();
      poses.add(
          new Pose2d(node.waypoint().position(), rotation != null ? rotation : Rotation2d.ZERO));
    }
    return poses;
  }
}
