package com.pathplanner.lib.path2;

import com.pathplanner.lib.util.FlippingUtil;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A waypoint in a 2027 path graph. A waypoint is one of three kinds, matching the waypoint types of
 * the PathPlanner app:
 *
 * <ul>
 *   <li>Pose waypoint: {@code rotation} is set and {@code pointTowardsTarget} is null. The robot
 *       turns to this heading while driving to the waypoint.
 *   <li>Translation waypoint: both are null. The robot keeps the heading of the next pose waypoint,
 *       or holds its current heading if none follows.
 *   <li>Point towards waypoint: {@code pointTowardsTarget} is set. The robot faces the target, plus
 *       {@code pointTowardsRotationOffset}, while driving to the waypoint.
 * </ul>
 *
 * @param position Field position of the waypoint from the blue alliance origin, in meters
 * @param rotation Heading of a pose waypoint, otherwise null
 * @param pointTowardsTarget Blue-origin field position a point towards waypoint faces, otherwise
 *     null
 * @param pointTowardsRotationOffset Offset added to the heading towards the target
 * @param unprofiled Use an unprofiled heading controller while aiming at the target
 * @param maxVelocityMPS Max linear velocity while driving to this waypoint, in meters per second
 * @param maxAngularVelocityRadPerSec Max angular velocity while driving to this waypoint, in
 *     radians per second
 * @param maxAngularAccelerationRadPerSecSq Max angular acceleration while driving to this waypoint,
 *     in radians per second squared
 * @param events Names of the events at this waypoint
 */
public record GraphWaypoint(
    Translation2d position,
    Rotation2d rotation,
    Translation2d pointTowardsTarget,
    Rotation2d pointTowardsRotationOffset,
    boolean unprofiled,
    double maxVelocityMPS,
    double maxAngularVelocityRadPerSec,
    double maxAngularAccelerationRadPerSecSq,
    List<String> events) {
  /** Max linear velocity used when a file leaves it out, matching the app's default */
  public static final double DEFAULT_MAX_VELOCITY_MPS = 3.0;

  /** Max angular velocity used when a file leaves it out, matching the app's default */
  public static final double DEFAULT_MAX_ANGULAR_VELOCITY_DEG = 540.0;

  /** Max angular acceleration used when a file leaves it out, matching the app's default */
  public static final double DEFAULT_MAX_ANGULAR_ACCELERATION_DEG = 720.0;

  /**
   * Create a waypoint
   *
   * @param position Field position of the waypoint from the blue alliance origin, in meters
   * @param rotation Heading of a pose waypoint, otherwise null
   * @param pointTowardsTarget Blue-origin field position a point towards waypoint faces, otherwise
   *     null
   * @param pointTowardsRotationOffset Offset added to the heading towards the target
   * @param unprofiled Use an unprofiled heading controller while aiming at the target
   * @param maxVelocityMPS Max linear velocity, in meters per second
   * @param maxAngularVelocityRadPerSec Max angular velocity, in radians per second
   * @param maxAngularAccelerationRadPerSecSq Max angular acceleration, in radians per second
   *     squared
   * @param events Names of the events at this waypoint
   */
  public GraphWaypoint {
    if (rotation != null && pointTowardsTarget != null) {
      throw new IllegalArgumentException(
          "A waypoint cannot have both a heading and a point towards target");
    }
    if (pointTowardsRotationOffset == null) {
      pointTowardsRotationOffset = Rotation2d.ZERO;
    }
    events = List.copyOf(events);
  }

  /**
   * Is this a pose waypoint
   *
   * @return True if this waypoint has a heading
   */
  public boolean isPose() {
    return rotation != null;
  }

  /**
   * Is this a point towards waypoint
   *
   * @return True if this waypoint faces a target
   */
  public boolean isPointTowards() {
    return pointTowardsTarget != null;
  }

  /**
   * Flip this waypoint to the other side of the field
   *
   * @return The flipped waypoint
   */
  public GraphWaypoint flip() {
    // Rotating the field keeps an offset relative to the target, and mirroring it reverses one
    Rotation2d flippedOffset =
        FlippingUtil.symmetryType == FlippingUtil.FieldSymmetry.kMirrored
            ? pointTowardsRotationOffset.unaryMinus()
            : pointTowardsRotationOffset;
    return new GraphWaypoint(
        FlippingUtil.flipFieldPosition(position),
        rotation == null ? null : FlippingUtil.flipFieldRotation(rotation),
        pointTowardsTarget == null ? null : FlippingUtil.flipFieldPosition(pointTowardsTarget),
        flippedOffset,
        unprofiled,
        maxVelocityMPS,
        maxAngularVelocityRadPerSec,
        maxAngularAccelerationRadPerSecSq,
        events);
  }

  /**
   * Mirror this waypoint to the other side of the current alliance, as {@link
   * com.pathplanner.lib.path.PathPlannerPath#mirrorPath()} does
   *
   * @return The mirrored waypoint
   */
  public GraphWaypoint mirror() {
    return new GraphWaypoint(
        mirrorPosition(position),
        rotation == null ? null : rotation.unaryMinus(),
        pointTowardsTarget == null ? null : mirrorPosition(pointTowardsTarget),
        pointTowardsRotationOffset.unaryMinus(),
        unprofiled,
        maxVelocityMPS,
        maxAngularVelocityRadPerSec,
        maxAngularAccelerationRadPerSecSq,
        events);
  }

  /**
   * Mirror a blue-origin field pose to the other side of the current alliance
   *
   * @param pose The pose to mirror
   * @return The mirrored pose
   */
  public static Pose2d mirrorPose(Pose2d pose) {
    return new Pose2d(mirrorPosition(pose.getTranslation()), pose.getRotation().unaryMinus());
  }

  private static Translation2d mirrorPosition(Translation2d position) {
    return new Translation2d(position.getX(), FlippingUtil.fieldSizeY - position.getY());
  }
}
