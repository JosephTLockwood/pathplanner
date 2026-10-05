package com.pathplanner.lib.command3;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.follower.PathFollower;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.Timer;

/**
 * Commands v3 command for following a path.
 *
 * <p>The commands of the path's event markers are forked as child commands while the path is
 * followed, so they will be canceled when the path ends. These commands may not require the drive
 * mechanisms.
 */
public class FollowPathCommand implements Command {
  private final PathFollower follower;
  private final EventScheduler eventScheduler;
  private final Set<Mechanism> requirements;
  private final String name;

  /**
   * Construct a path following command
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field? This will
   *     maintain a global blue alliance origin.
   * @param requirements Mechanisms required by this command, usually just the drive mechanism
   */
  public FollowPathCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Mechanism... requirements) {
    this.follower =
        new PathFollower(
            path, poseSupplier, speedsSupplier, output, controller, robotConfig, shouldFlipPath);
    this.eventScheduler = new EventScheduler();
    this.requirements = Set.of(requirements);
    this.name = nameWithPath("Follow Path", path);

    var eventReqs = eventScheduler.buildEventCommands(path);
    if (!Collections.disjoint(this.requirements, eventReqs)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive mechanism");
    }
  }

  @Override
  public void run(Coroutine coroutine) {
    PathPlannerTrajectory trajectory = follower.start();
    Timer timer = Timer.createStarted();

    // Handle the trajectory's events alongside this command. The events command is a child of this
    // command, so it (and any event commands it started) will end when this command ends.
    coroutine.fork(eventScheduler.eventsCommand(trajectory, timer::get));

    double time = timer.get();
    follower.follow(time);
    while (!follower.isFinished(time)) {
      coroutine.yield();

      time = timer.get();
      follower.follow(time);
    }

    follower.stop(false);
  }

  @Override
  public void onCancel() {
    follower.stop(true);
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return requirements;
  }

  @Override
  public String toString() {
    return name();
  }

  /**
   * Build a command name that includes the name of a path, if the path has one
   *
   * @param prefix The start of the command name
   * @param path The path
   * @return The command name
   */
  static String nameWithPath(String prefix, PathPlannerPath path) {
    if (path.name == null || path.name.isEmpty()) {
      return prefix;
    }
    return prefix + ": " + path.name;
  }

  /**
   * Create a command to warmup on-the-fly generation, replanning, and the path following command
   *
   * @return Path following warmup command
   */
  public static Command warmupCommand() {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(0.0, 0.0, Rotation2d.ZERO), new Pose2d(6.0, 6.0, Rotation2d.ZERO));
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            new PathConstraints(4.0, 4.0, 4.0, 4.0),
            new IdealStartingState(0.0, Rotation2d.ZERO),
            new GoalEndState(0.0, Rotation2d.CW_90DEG));

    Command followPath =
        new FollowPathCommand(
            path,
            () -> Pose2d.ZERO,
            ChassisVelocities::new,
            (speeds, feedforwards) -> {},
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            new RobotConfig(
                75,
                6.8,
                new ModuleConfig(
                    0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                0.55),
            () -> true);

    return Command.noRequirements(
            coroutine -> {
              coroutine.await(followPath);
              System.out.println("[PathPlanner] FollowPathCommand finished warmup");
            })
        .named("FollowPathCommand Warmup");
  }
}
