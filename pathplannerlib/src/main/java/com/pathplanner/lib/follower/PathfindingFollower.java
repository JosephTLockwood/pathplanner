package com.pathplanner.lib.follower;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.*;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;

/**
 * Pathfinds to a target pose or path, and follows the generated path. This contains the pathfinding
 * logic shared by the pathfinding commands of each supported command framework, and does not depend
 * on a command framework itself.
 *
 * <p>Pathfinding is done by calling {@link #start()}, then calling {@link #update()} every loop
 * until {@link #isFinished()} returns true, then calling {@link #stop(boolean)}.
 */
public class PathfindingFollower {
  private final Timer timer = new Timer();
  private final PathPlannerPath targetPath;
  private Pose2d targetPose;
  private Pose2d originalTargetPose;
  private GoalEndState goalEndState;
  private final PathConstraints constraints;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisVelocities> speedsSupplier;
  private final BiConsumer<ChassisVelocities, DriveFeedforwards> output;
  private final PathFollowingController controller;
  private final RobotConfig robotConfig;
  private final BooleanSupplier shouldFlipPath;

  private PathPlannerTrajectory currentTrajectory;
  private double timeOffset = 0;
  private boolean finish = false;
  private boolean running = false;

  /**
   * Constructs a new pathfinding follower that will generate a path towards the given path.
   *
   * @param targetPath the path to pathfind to
   * @param constraints the path constraints to use while pathfinding
   * @param poseSupplier a supplier for the robot's current pose
   * @param speedsSupplier a supplier for the robot's current robot relative speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the target path be flipped to the other side of the field? This
   *     will maintain a global blue alliance origin.
   */
  public PathfindingFollower(
      PathPlannerPath targetPath,
      PathConstraints constraints,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath) {
    Pathfinding.ensureInitialized();

    Rotation2d targetRotation = Rotation2d.ZERO;
    double goalEndVel = targetPath.getGlobalConstraints().maxVelocityMPS();
    if (targetPath.isChoreoPath()) {
      // Can get() here without issue since all choreo trajectories have ideal trajectories
      PathPlannerTrajectory choreoTraj = targetPath.getIdealTrajectory(robotConfig).orElseThrow();
      targetRotation = choreoTraj.getInitialState().pose.getRotation();
      goalEndVel = choreoTraj.getInitialState().linearVelocity;
    } else {
      for (PathPoint p : targetPath.getAllPathPoints()) {
        if (p.rotationTarget != null) {
          targetRotation = p.rotationTarget.rotation();
          break;
        }
      }
    }

    this.targetPath = targetPath;
    this.targetPose = new Pose2d(this.targetPath.getPoint(0).position, targetRotation);
    this.originalTargetPose =
        new Pose2d(this.targetPose.getTranslation(), this.targetPose.getRotation());
    this.goalEndState = new GoalEndState(goalEndVel, targetRotation);
    this.constraints = constraints;
    this.controller = controller;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = output;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;
  }

  /**
   * Constructs a new pathfinding follower that will generate a path towards the given pose.
   *
   * @param targetPose the pose to pathfind to, the rotation component is only relevant for
   *     holonomic drive trains
   * @param constraints the path constraints to use while pathfinding
   * @param goalEndVel The goal end velocity when reaching the target pose
   * @param poseSupplier a supplier for the robot's current pose
   * @param speedsSupplier a supplier for the robot's current robot relative speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   */
  public PathfindingFollower(
      Pose2d targetPose,
      PathConstraints constraints,
      double goalEndVel,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig) {
    Pathfinding.ensureInitialized();

    this.targetPath = null;
    this.targetPose = targetPose;
    this.originalTargetPose =
        new Pose2d(this.targetPose.getTranslation(), this.targetPose.getRotation());
    this.goalEndState = new GoalEndState(goalEndVel, targetPose.getRotation());
    this.constraints = constraints;
    this.controller = controller;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = output;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = () -> false;
  }

  /** Start pathfinding from the robot's current pose. */
  public void start() {
    currentTrajectory = null;
    timeOffset = 0;
    finish = false;
    running = true;

    Pose2d currentPose = poseSupplier.get();

    controller.reset(currentPose, speedsSupplier.get());

    if (targetPath != null) {
      originalTargetPose =
          new Pose2d(this.targetPath.getPoint(0).position, originalTargetPose.getRotation());
      if (shouldFlipPath.getAsBoolean()) {
        targetPose = FlippingUtil.flipFieldPose(this.originalTargetPose);
        goalEndState = new GoalEndState(goalEndState.velocityMPS(), targetPose.getRotation());
      }
    }

    if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.5) {
      output.accept(new ChassisVelocities(), DriveFeedforwards.zeros(robotConfig.numModules));
      finish = true;
    } else {
      Pathfinding.setStartPosition(currentPose.getTranslation());
      Pathfinding.setGoalPosition(targetPose.getTranslation());
    }
  }

  /**
   * Update the pathfinding path if a new one is available, and drive the robot along it. This
   * should be called every loop while pathfinding.
   */
  public void update() {
    if (finish) {
      return;
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisVelocities currentSpeeds = speedsSupplier.get();

    PathPlannerLogging.logCurrentPose(currentPose);
    PPLibTelemetry.setCurrentPose(currentPose);

    // Skip new paths if we are close to the end
    boolean skipUpdates =
        currentTrajectory != null
            && currentPose
                    .getTranslation()
                    .getDistance(currentTrajectory.getEndState().pose.getTranslation())
                < 2.0;

    if (!skipUpdates && Pathfinding.isNewPathAvailable()) {
      PathPlannerPath currentPath = Pathfinding.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        currentTrajectory =
            new PathPlannerTrajectory(
                currentPath, currentSpeeds, currentPose.getRotation(), robotConfig);

        if (!Double.isFinite(currentTrajectory.getTotalTimeSeconds())) {
          finish = true;
          return;
        }

        // Find the two closest states in front of and behind robot
        int closestState1Idx = 0;
        int closestState2Idx = 1;
        while (closestState2Idx < currentTrajectory.getStates().size() - 1) {
          double closest2Dist =
              currentTrajectory
                  .getState(closestState2Idx)
                  .pose
                  .getTranslation()
                  .getDistance(currentPose.getTranslation());
          double nextDist =
              currentTrajectory
                  .getState(closestState2Idx + 1)
                  .pose
                  .getTranslation()
                  .getDistance(currentPose.getTranslation());
          if (nextDist < closest2Dist) {
            closestState1Idx++;
            closestState2Idx++;
          } else {
            break;
          }
        }

        // Use the closest 2 states to interpolate what the time offset should be
        // This will account for the delay in pathfinding
        var closestState1 = currentTrajectory.getState(closestState1Idx);
        var closestState2 = currentTrajectory.getState(closestState2Idx);

        double d =
            closestState1.pose.getTranslation().getDistance(closestState2.pose.getTranslation());
        double t =
            (currentPose.getTranslation().getDistance(closestState1.pose.getTranslation())) / d;
        t = Math.clamp(t, 0.0, 1.0);

        timeOffset = MathUtil.lerp(closestState1.timeSeconds, closestState2.timeSeconds, t);

        // If the robot is stationary and at the start of the path, set the time offset to the next
        // loop
        // This can prevent an issue where the robot will remain stationary if new paths come in
        // every loop
        if (timeOffset <= 0.02 && Math.hypot(currentSpeeds.vx, currentSpeeds.vy) < 0.1) {
          timeOffset = 0.02;
        }

        PathPlannerLogging.logActivePath(currentPath);
        PPLibTelemetry.setCurrentPath(currentPath);
      }

      timer.reset();
      timer.start();
    }

    if (currentTrajectory != null) {
      var targetState = currentTrajectory.sample(timer.get() + timeOffset);
      PathFollower.driveToState(controller, currentPose, currentSpeeds, targetState, output);
    }
  }

  /**
   * Check if pathfinding has finished
   *
   * @return True if the robot has reached the target
   */
  public boolean isFinished() {
    if (finish) {
      return true;
    }

    if (targetPath != null && !targetPath.isChoreoPath()) {
      Pose2d currentPose = poseSupplier.get();
      ChassisVelocities currentSpeeds = speedsSupplier.get();

      double currentVel = Math.hypot(currentSpeeds.vx, currentSpeeds.vy);
      double stoppingDistance = Math.pow(currentVel, 2) / (2 * constraints.maxAccelerationMPSSq());

      return currentPose.getTranslation().getDistance(targetPose.getTranslation())
          <= stoppingDistance;
    }

    if (currentTrajectory != null) {
      return timer.hasElapsed(currentTrajectory.getTotalTimeSeconds() - timeOffset);
    }

    return false;
  }

  /**
   * Stop pathfinding. Does nothing if pathfinding has not been started.
   *
   * @param interrupted True if pathfinding was interrupted before it was finished
   */
  public void stop(boolean interrupted) {
    if (!running) {
      return;
    }
    running = false;

    timer.stop();

    // Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
    // the command to smoothly transition into some auto-alignment routine
    if (!interrupted && goalEndState.velocityMPS() < 0.1) {
      output.accept(new ChassisVelocities(), DriveFeedforwards.zeros(robotConfig.numModules));
    }

    PathPlannerLogging.logActivePath(null);
  }

  /**
   * Create a path that joins the robot's current state to the start of a goal path. This is used
   * after pathfinding to a path, to smoothly connect the end of the pathfinding path to the start
   * of the goal path.
   *
   * @param goalPath The path that will be followed after the join path
   * @param pathfindingConstraints The constraints to use for the join path
   * @param startPose The current pose of the robot
   * @param startSpeeds The current robot relative speeds of the robot
   * @param shouldFlipPath Should the goal path be flipped to the other side of the field
   * @return The join path, or empty if the goal path does not have enough points to join to
   */
  public static Optional<PathPlannerPath> createJoinPath(
      PathPlannerPath goalPath,
      PathConstraints pathfindingConstraints,
      Pose2d startPose,
      ChassisVelocities startSpeeds,
      boolean shouldFlipPath) {
    if (goalPath.numPoints() < 2) {
      return Optional.empty();
    }

    ChassisVelocities startFieldSpeeds = startSpeeds.toFieldRelative(startPose.getRotation());
    Rotation2d startHeading = new Rotation2d(startFieldSpeeds.vx, startFieldSpeeds.vy);

    Pose2d endWaypoint = new Pose2d(goalPath.getPoint(0).position, goalPath.getInitialHeading());
    boolean shouldFlip = shouldFlipPath && !goalPath.preventFlipping;
    if (shouldFlip) {
      endWaypoint = FlippingUtil.flipFieldPose(endWaypoint);
    }

    GoalEndState endState;
    if (goalPath.getIdealStartingState() != null) {
      Rotation2d endRot = goalPath.getIdealStartingState().rotation();
      if (shouldFlip) {
        endRot = FlippingUtil.flipFieldRotation(endRot);
      }
      endState = new GoalEndState(goalPath.getIdealStartingState().velocityMPS(), endRot);
    } else {
      endState = new GoalEndState(pathfindingConstraints.maxVelocityMPS(), startPose.getRotation());
    }

    PathPlannerPath joinPath =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(startPose.getTranslation(), startHeading), endWaypoint),
            pathfindingConstraints,
            new IdealStartingState(
                Math.hypot(startSpeeds.vx, startSpeeds.vy), startPose.getRotation()),
            endState);
    joinPath.preventFlipping = true;

    return Optional.of(joinPath);
  }
}
