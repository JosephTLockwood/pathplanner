package com.pathplanner.lib.commands;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.follower.ActivePathState;
import com.pathplanner.lib.follower.PathFollower;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.DriveFeedforwards;
import java.util.*;
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
import org.wpilib.system.Timer;

/** Base command for following a path */
public class FollowPathCommand extends Command {

  private final Timer timer = new Timer();

  private final PathFollower follower;

  private final EventScheduler eventScheduler;

  /**
   * Construct a base path following command
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
   * @param requirements Subsystems required by this command, usually just the drive subsystem
   */
  public FollowPathCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... requirements) {
    this.follower =
        new PathFollower(
            path, poseSupplier, speedsSupplier, output, controller, robotConfig, shouldFlipPath);
    this.eventScheduler = new EventScheduler();

    Set<Subsystem> driveRequirements = Set.of(requirements);
    addRequirements(requirements);

    // Add all event scheduler requirements to this command's requirements
    var eventReqs = eventScheduler.buildEventCommands(path);
    if (!Collections.disjoint(driveRequirements, eventReqs)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventReqs);
  }

  @Override
  public void initialize() {
    eventScheduler.initialize(follower.start());
    PathPlannerAuto.currentPathName = ActivePathState.getCurrentPathName();

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double currentTime = timer.get();

    follower.follow(currentTime);
    eventScheduler.execute(currentTime);
  }

  @Override
  public boolean isFinished() {
    return follower.isFinished(timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    follower.stop(interrupted);
    PathPlannerAuto.currentPathName = "";
    eventScheduler.end();
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
    return new FollowPathCommand(
            path,
            () -> Pose2d.ZERO,
            ChassisVelocities::new,
            (_, _) -> {},
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            new RobotConfig(
                75,
                6.8,
                new ModuleConfig(
                    0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                0.55),
            () -> true)
        .andThen(Commands.print("[PathPlanner] FollowPathCommand finished warmup"))
        .ignoringDisable(true);
  }
}
