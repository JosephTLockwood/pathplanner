package com.pathplanner.lib.command3;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path2.GraphFollower;
import com.pathplanner.lib.path2.PathGraph;
import com.pathplanner.lib.util.DriveFeedforwards;
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
 * Commands v3 command for following a path drawn in the PathPlanner 2027 app. See {@link
 * GraphFollower} for how the path is followed, how branches are chosen, and when events fire.
 *
 * <p>When the robot reaches the end waypoint, the command sends zero speeds and ends. When the
 * command is canceled, it sends nothing, so the last speeds it sent stay in effect until something
 * else drives the robot.
 */
public class FollowPathGraphCommand implements Command {
  private final GraphFollower follower;
  private final Set<Mechanism> requirements;
  private final String name;

  /**
   * Construct a path following command
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot,
   *     measured from the center of the field as the 2027 app draws it
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
   *     each swerve module, in FL, FR, BL, BR order
   * @param robotConfig The robot configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field
   * @param requirements Mechanisms required by this command, usually just the drive mechanism
   */
  public FollowPathGraphCommand(
      PathGraph path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisVelocities> speedsSupplier,
      BiConsumer<ChassisVelocities, DriveFeedforwards> output,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Mechanism... requirements) {
    this.follower =
        new GraphFollower(path, poseSupplier, speedsSupplier, output, robotConfig, shouldFlipPath);
    this.requirements = Set.of(requirements);
    this.name = path.name.isEmpty() ? "Follow Path" : "Follow Path: " + path.name;
  }

  @Override
  public void run(Coroutine coroutine) {
    follower.start();
    follower.follow();
    while (!follower.isFinished()) {
      coroutine.yield();
      follower.follow();
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
}
