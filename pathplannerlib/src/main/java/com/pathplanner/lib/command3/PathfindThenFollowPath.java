package com.pathplanner.lib.command3;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.follower.PathfindingFollower;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Commands v3 command that pathfinds to the start of a path, joins the end of the pathfinding path
 * to the start of the path, then follows the path.
 */
public class PathfindThenFollowPath implements Command {
  private final PathPlannerPath goalPath;
  private final PathConstraints pathfindingConstraints;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisVelocities> speedsSupplier;
  private final BiConsumer<ChassisVelocities, DriveFeedforwards> output;
  private final PathFollowingController controller;
  private final RobotConfig robotConfig;
  private final BooleanSupplier shouldFlipPath;
  private final Mechanism[] driveRequirements;

  private final Command pathfindToPath;
  private final Command followGoalPath;
  private final Set<Mechanism> requirements;
  private final String name;

  /**
   * Constructs a new PathfindThenFollowPath command.
   *
   * @param goalPath the goal path to follow
   * @param pathfindingConstraints the path constraints for pathfinding
   * @param poseSupplier a supplier for the robot's current pose
   * @param currentRobotRelativeSpeeds a supplier for the robot's current robot relative speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the target path be flipped to the other side of the field? This
   *     will maintain a global blue alliance origin.
   * @param requirements the mechanisms required by this command (drive mechanism)
   */
  public PathfindThenFollowPath(
      PathPlannerPath goalPath,
      PathConstraints pathfindingConstraints,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> currentRobotRelativeSpeeds,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Mechanism... requirements) {
    this.goalPath = goalPath;
    this.pathfindingConstraints = pathfindingConstraints;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = currentRobotRelativeSpeeds;
    this.output = output;
    this.controller = controller;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;
    this.driveRequirements = requirements;

    this.pathfindToPath =
        new PathfindingCommand(
            goalPath,
            pathfindingConstraints,
            poseSupplier,
            currentRobotRelativeSpeeds,
            output,
            controller,
            robotConfig,
            shouldFlipPath,
            requirements);
    this.followGoalPath = followPathCommand(goalPath);
    this.requirements = Set.of(requirements);
    this.name = FollowPathCommand.nameWithPath("Pathfind Then Follow Path", goalPath);
  }

  @Override
  public void run(Coroutine coroutine) {
    coroutine.await(pathfindToPath);

    // Generate an on-the-fly path to join the end of the pathfinding path to the start of the goal
    // path, now that the robot's state at the end of pathfinding is known
    Optional<PathPlannerPath> joinPath =
        PathfindingFollower.createJoinPath(
            goalPath,
            pathfindingConstraints,
            poseSupplier.get(),
            speedsSupplier.get(),
            shouldFlipPath.getAsBoolean());
    if (joinPath.isPresent()) {
      coroutine.await(followPathCommand(joinPath.get()));
    }

    coroutine.await(followGoalPath);
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

  private Command followPathCommand(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        poseSupplier,
        speedsSupplier,
        output,
        controller,
        robotConfig,
        shouldFlipPath,
        driveRequirements);
  }
}
