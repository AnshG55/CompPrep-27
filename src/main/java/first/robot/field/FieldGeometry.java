package first.robot.field;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Where things are on the field, and how to turn "our side" positions into real field positions.
 *
 * <p>Field coordinates always have the origin at the blue alliance wall's right corner (as seen by
 * the blue drivers), +x pointing toward red, +y to the blue drivers' left. That doesn't change with
 * the alliance: a red robot at its own wall has an x near the field length. So positions here are
 * written once, from blue's side, and {@link #flipIfRed} moves them to red's side when needed.
 *
 * <p>Everything that depends on where the origin is lives in this file. If a future season moves
 * the origin (to field center, say), this is the file to change.
 */
public final class FieldGeometry {
  private FieldGeometry() {}

  /**
   * The field and its AprilTags. 2027 moved tag layouts from AprilTagFieldLayout to the Field
   * class.
   */
  public static final Field FIELD = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

  public static final double LENGTH_METERS = FIELD.getFieldLength();
  public static final double WIDTH_METERS = FIELD.getFieldWidth();

  /** True when the Driver Station says we're red. Unknown counts as blue. */
  public static boolean isRed() {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  /**
   * The heading that faces away from our drivers, toward the other alliance: 0 degrees on blue, 180
   * on red.
   */
  public static Rotation2d downfield() {
    return isRed() ? Rotation2d.k180deg : Rotation2d.ZERO;
  }

  /**
   * The same spot on the other alliance's side. The 2026 field is rotationally symmetric: red's
   * half is blue's half turned 180 degrees about the field center, so x, y, and heading all flip.
   * (Some years mirror the field left to right instead. Then only x and heading flip.)
   */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), pose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(LENGTH_METERS - translation.getX(), WIDTH_METERS - translation.getY());
  }

  /** A blue-side pose moved to our side of the field. */
  public static Pose2d flipIfRed(Pose2d bluePose) {
    return isRed() ? flip(bluePose) : bluePose;
  }

  /** The heading that points the robot's front from {@code robot} at {@code point}. */
  public static Rotation2d headingToward(Pose2d robot, Translation2d point) {
    var offset = point.minus(robot.getTranslation());
    // Standing exactly on the point there's no direction to face, so keep the current heading.
    return offset.getAngle().orElse(robot.getRotation());
  }
}
