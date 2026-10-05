package com.pathplanner.lib.follower;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Follows a PathPlannerPath. This contains the path following logic shared by the path following
 * commands of each supported command framework, and does not depend on a command framework itself.
 *
 * <p>A path is followed by calling {@link #start()}, then calling {@link #follow(double)} every
 * loop until {@link #isFinished(double)} returns true, then calling {@link #stop(boolean)}.
 */
public class PathFollower {
  private final PathPlannerPath originalPath;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisVelocities> speedsSupplier;
  private final BiConsumer<ChassisVelocities, DriveFeedforwards> output;
  private final PathFollowingController controller;
  private final RobotConfig robotConfig;
  private final BooleanSupplier shouldFlipPath;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;
  private boolean following = false;

  /**
   * Construct a path follower
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field? This will
   *     maintain a global blue alliance origin.
   */
  public PathFollower(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath) {
    this.originalPath = path;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = output;
    this.controller = controller;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;

    this.path = this.originalPath;
    // Ensure the ideal trajectory is generated
    this.path.getIdealTrajectory(this.robotConfig).ifPresent(traj -> this.trajectory = traj);
  }

  /**
   * Start following the path. This flips the path if needed and generates the trajectory to follow
   * from the robot's current state.
   *
   * @return The trajectory that will be followed
   */
  public PathPlannerTrajectory start() {
    if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
      path = originalPath.flipPath();
    } else {
      path = originalPath;
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisVelocities currentSpeeds = speedsSupplier.get();

    controller.reset(currentPose, currentSpeeds);

    double linearVel = Math.hypot(currentSpeeds.vx, currentSpeeds.vy);

    if (path.getIdealStartingState() != null) {
      // Check if we match the ideal starting state
      boolean idealVelocity =
          Math.abs(linearVel - path.getIdealStartingState().velocityMPS()) <= 0.25;
      boolean idealRotation =
          !robotConfig.isHolonomic
              || Math.abs(
                      currentPose
                          .getRotation()
                          .minus(path.getIdealStartingState().rotation())
                          .getDegrees())
                  <= 30.0;
      if (idealVelocity && idealRotation) {
        // We can use the ideal trajectory
        trajectory = path.getIdealTrajectory(robotConfig).orElseThrow();
      } else {
        // We need to regenerate
        trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
      }
    } else {
      // No ideal starting state, generate the trajectory
      trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
    }

    ActivePathState.setCurrentTrajectory(trajectory);
    ActivePathState.setCurrentPathName(originalPath.name);

    PathPlannerLogging.logActivePath(path);
    PPLibTelemetry.setCurrentPath(path);

    following = true;
    return trajectory;
  }

  /**
   * Drive the robot along the trajectory. This should be called every loop while following the
   * path.
   *
   * @param time The current time along the trajectory, in seconds
   */
  public void follow(double time) {
    var targetState = trajectory.sample(time);
    if (!controller.isHolonomic() && path.isReversed()) {
      targetState = targetState.reverse();
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisVelocities currentSpeeds = speedsSupplier.get();

    ChassisVelocities targetSpeeds =
        controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    double currentVel = Math.hypot(currentSpeeds.vx, currentSpeeds.vy);

    PPLibTelemetry.setCurrentPose(currentPose);
    PathPlannerLogging.logCurrentPose(currentPose);

    PPLibTelemetry.setTargetPose(targetState.pose);
    PathPlannerLogging.logTargetPose(targetState.pose);

    PPLibTelemetry.setVelocities(
        currentVel, targetState.linearVelocity, currentSpeeds.omega, targetSpeeds.omega);

    output.accept(targetSpeeds, targetState.feedforwards);
  }

  /**
   * Check if the trajectory has been completed
   *
   * @param time The current time along the trajectory, in seconds
   * @return True if the end of the trajectory has been reached
   */
  public boolean isFinished(double time) {
    double totalTime = trajectory.getTotalTimeSeconds();
    return time >= totalTime || !Double.isFinite(totalTime);
  }

  /**
   * Stop following the path. Does nothing if the path is not being followed.
   *
   * @param interrupted True if the path was interrupted before it was finished
   */
  public void stop(boolean interrupted) {
    if (!following) {
      return;
    }
    following = false;

    ActivePathState.setCurrentPathName("");
    ActivePathState.setCurrentTrajectory(null);

    // Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
    // the command to smoothly transition into some auto-alignment routine
    if (!interrupted && path.getGoalEndState().velocityMPS() < 0.1) {
      output.accept(new ChassisVelocities(), DriveFeedforwards.zeros(robotConfig.numModules));
    }

    PathPlannerLogging.logActivePath(null);
  }

  /**
   * Get the path given to this follower, before any flipping
   *
   * @return The original path
   */
  public PathPlannerPath getOriginalPath() {
    return originalPath;
  }

  /**
   * Get the trajectory being followed. This is the ideal trajectory until {@link #start()} is
   * called.
   *
   * @return The trajectory, or null if it has not been generated yet
   */
  public PathPlannerTrajectory getTrajectory() {
    return trajectory;
  }
}
