package com.pathplanner.lib.path2;

import com.pathplanner.lib.util.FlippingUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Flips positions and rotations measured from the center of the field. 2027 path and auto files use
 * this coordinate system: the origin is the center of the field, and +X points away from the red
 * alliance wall. The type of symmetry is read from {@link FlippingUtil#symmetryType}.
 */
public final class CenterFieldFlipping {
  private CenterFieldFlipping() {}

  /**
   * Flip a center-origin field position to the other alliance
   *
   * @param pos The position to flip
   * @return The flipped position
   */
  public static Translation2d flipPosition(Translation2d pos) {
    return switch (FlippingUtil.symmetryType) {
      case kMirrored -> new Translation2d(-pos.getX(), pos.getY());
      case kRotational -> new Translation2d(-pos.getX(), -pos.getY());
    };
  }

  /**
   * Flip a field rotation to the other alliance
   *
   * @param rotation The rotation to flip
   * @return The flipped rotation
   */
  public static Rotation2d flipRotation(Rotation2d rotation) {
    return switch (FlippingUtil.symmetryType) {
      case kMirrored -> Rotation2d.PI.minus(rotation);
      case kRotational -> rotation.minus(Rotation2d.PI);
    };
  }

  /**
   * Flip a rotation measured relative to another direction, such as a point towards rotation
   * offset. Rotating the field keeps relative angles, and mirroring it reverses them.
   *
   * @param offset The relative rotation to flip
   * @return The flipped relative rotation
   */
  public static Rotation2d flipRotationOffset(Rotation2d offset) {
    return switch (FlippingUtil.symmetryType) {
      case kMirrored -> offset.unaryMinus();
      case kRotational -> offset;
    };
  }

  /**
   * Flip a center-origin field pose to the other alliance
   *
   * @param pose The pose to flip
   * @return The flipped pose
   */
  public static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(flipPosition(pose.getTranslation()), flipRotation(pose.getRotation()));
  }

  /**
   * Mirror a center-origin field position to the other side of the current alliance
   *
   * @param pos The position to mirror
   * @return The mirrored position
   */
  public static Translation2d mirrorPosition(Translation2d pos) {
    return new Translation2d(pos.getX(), -pos.getY());
  }

  /**
   * Mirror a center-origin field pose to the other side of the current alliance
   *
   * @param pose The pose to mirror
   * @return The mirrored pose
   */
  public static Pose2d mirrorPose(Pose2d pose) {
    return new Pose2d(mirrorPosition(pose.getTranslation()), pose.getRotation().unaryMinus());
  }
}
