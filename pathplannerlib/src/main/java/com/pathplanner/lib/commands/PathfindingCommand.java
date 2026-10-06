package com.pathplanner.lib.commands;

import static org.wpilib.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.follower.PathfindingFollower;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.*;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.util.UsageReporting;

/** Base pathfinding command */
public class PathfindingCommand extends Command {

  private static int instances = 0;

  private final PathfindingFollower follower;

  /**
   * Constructs a new base pathfinding command that will generate a path towards the given path.
   *
   * @param targetPath the path to pathfind to
   * @param constraints the path constraints to use while pathfinding
   * @param poseSupplier a supplier for the robot's current pose
   * @param speedsSupplier a supplier for the robot's current robot relative speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the target path be flipped to the other side of the field? This
   *     will maintain a global blue alliance origin.
   * @param requirements the subsystems required by this command
   */
  public PathfindingCommand(
      PathPlannerPath targetPath,
      PathConstraints constraints,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... requirements) {
    addRequirements(requirements);

    this.follower =
        new PathfindingFollower(
            targetPath,
            constraints,
            poseSupplier,
            speedsSupplier,
            output,
            controller,
            robotConfig,
            shouldFlipPath);

    instances++;
    UsageReporting.reportUsage("PathPlanner/PathFindingCommand", instances, "");
  }

  /**
   * Constructs a new base pathfinding command that will generate a path towards the given pose.
   *
   * @param targetPose the pose to pathfind to, the rotation component is only relevant for
   *     holonomic drive trains
   * @param constraints the path constraints to use while pathfinding
   * @param goalEndVel The goal end velocity when reaching the target pose
   * @param poseSupplier a supplier for the robot's current pose
   * @param speedsSupplier a supplier for the robot's current robot relative speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param requirements the subsystems required by this command
   */
  public PathfindingCommand(
      Pose2d targetPose,
      PathConstraints constraints,
      double goalEndVel,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      Subsystem... requirements) {
    addRequirements(requirements);

    this.follower =
        new PathfindingFollower(
            targetPose,
            constraints,
            goalEndVel,
            poseSupplier,
            speedsSupplier,
            output,
            controller,
            robotConfig);

    instances++;
    UsageReporting.reportUsage("PathPlanner/PathFindingCommand", instances, "");
  }

  /**
   * Constructs a new base pathfinding command that will generate a path towards the given pose.
   *
   * @param targetPose the pose to pathfind to, the rotation component is only relevant for
   *     holonomic drive trains
   * @param constraints the path constraints to use while pathfinding
   * @param goalEndVel The goal end velocity when reaching the target pose
   * @param poseSupplier a supplier for the robot's current pose
   * @param speedsSupplier a supplier for the robot's current robot relative speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param requirements the subsystems required by this command
   */
  public PathfindingCommand(
      Pose2d targetPose,
      PathConstraints constraints,
      LinearVelocity goalEndVel,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      Subsystem... requirements) {
    this(
        targetPose,
        constraints,
        goalEndVel.in(MetersPerSecond),
        poseSupplier,
        speedsSupplier,
        output,
        controller,
        robotConfig,
        requirements);
  }

  /**
   * Constructs a new base pathfinding command that will generate a path towards the given pose.
   *
   * @param targetPose the pose to pathfind to, the rotation component is only relevant for
   *     holonomic drive trains
   * @param constraints the path constraints to use while pathfinding
   * @param poseSupplier a supplier for the robot's current pose
   * @param speedsSupplier a supplier for the robot's current robot relative speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
   *     using a differential drive, they will be in L, R order.
   *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
   *     module states, you will need to reverse the feedforwards for modules that have been flipped
   * @param controller Path following controller that will be used to follow the path
   * @param robotConfig The robot configuration
   * @param requirements the subsystems required by this command
   */
  public PathfindingCommand(
      Pose2d targetPose,
      PathConstraints constraints,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      Subsystem... requirements) {
    this(
        targetPose,
        constraints,
        0.0,
        poseSupplier,
        speedsSupplier,
        output,
        controller,
        robotConfig,
        requirements);
  }

  @Override
  public void initialize() {
    follower.start();
  }

  @Override
  public void execute() {
    follower.update();
  }

  @Override
  public boolean isFinished() {
    return follower.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    follower.stop(interrupted);
  }

  /**
   * Create a command to warmup the pathfinder and pathfinding command
   *
   * @return Pathfinding warmup command
   */
  public static Command warmupCommand() {
    return new PathfindingCommand(
            new Pose2d(15.0, 4.0, Rotation2d.k180deg),
            new PathConstraints(4, 3, 4, 4),
            () -> new Pose2d(1.5, 4, Rotation2d.ZERO),
            ChassisVelocities::new,
            (_, _) -> {},
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            new RobotConfig(
                75,
                6.8,
                new ModuleConfig(
                    0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                0.55))
        .andThen(Commands.print("[PathPlanner] PathfindingCommand finished warmup"))
        .ignoringDisable(true);
  }
}
