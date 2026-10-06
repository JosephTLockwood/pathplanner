package com.pathplanner.lib.commands;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.follower.PathfindingFollower;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Command group that will pathfind to the start of a path, then follow that path */
public class PathfindThenFollowPath extends SequentialCommandGroup {

  /**
   * Constructs a new PathfindThenFollowPath command group.
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
   * @param requirements the subsystems required by this command (drive subsystem)
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
      Subsystem... requirements) {
    // Use a deferred command to generate an on-the-fly path to join
    // Use a deferred command to generate an on-the-fly path to join
    // the end of the pathfinding command to the start of the path
    addCommands(
        new PathfindingCommand(
            goalPath,
            pathfindingConstraints,
            poseSupplier,
            currentRobotRelativeSpeeds,
            output,
            controller,
            robotConfig,
            shouldFlipPath,
            requirements),
        Commands.defer(
            () ->
                PathfindingFollower.createJoinPath(
                        goalPath,
                        pathfindingConstraints,
                        poseSupplier.get(),
                        currentRobotRelativeSpeeds.get(),
                        shouldFlipPath.getAsBoolean())
                    .<Command>map(
                        joinPath ->
                            new FollowPathCommand(
                                joinPath,
                                poseSupplier,
                                currentRobotRelativeSpeeds,
                                output,
                                controller,
                                robotConfig,
                                shouldFlipPath,
                                requirements))
                    .orElseGet(Commands::none),
            Set.of(requirements)),
        new FollowPathCommand(
            goalPath,
            poseSupplier,
            currentRobotRelativeSpeeds,
            output,
            controller,
            robotConfig,
            shouldFlipPath,
            requirements));
  }
}
