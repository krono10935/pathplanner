package com.pathplanner.lib.path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * where all the constants are stored. the static constants are the real constants that have default
 * value. the instanced constants are the pointers to DriveTrain.
 *
 * @param poseSupplier robot pose
 * @param speedsSupplier ChassisSpeeds supplier
 * @param drive ChassisSpeeds consumer
 * @param useDriveToPose if we would use DriveToPose command at the end of the path
 */
public record DriveToPoseConstants(
    Supplier<Pose2d> poseSupplier,
    Supplier<ChassisSpeeds> speedsSupplier,
    Consumer<ChassisSpeeds> drive,
    boolean useDriveToPose) {
  /** the max linear speed of the robot. */
  public static double MAX_LINEAR_SPEED = 4; // m/s
  /** the distance in which the auto should transfer from PP path to DriveToPose. */
  public static double DISTANCE_TO_STOP_PP = 0.751; // m
  /** the linear Trapezoid Profile constraints for autonomous */
  public static TrapezoidProfile.Constraints LINEAR_AUTO_CONSTRAINTS =
      new TrapezoidProfile.Constraints(4, 1);
  /** the linear Trapezoid Profile constraints for tele-op */
  public static TrapezoidProfile.Constraints LINEAR_TELE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(4, 1);
  /** the angle Trapezoid Profile constraints for autonomous */
  public static TrapezoidProfile.Constraints ANGLE_AUTO_CONSTRAINTS =
      new TrapezoidProfile.Constraints(9.753 / 1.2, Units.degreesToRadians(45));
  /** the angle Trapezoid Profile constraints for tele-op */
  public static TrapezoidProfile.Constraints ANGLE_TELE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(9.753 / 1.2, Units.degreesToRadians(45));
  /** linear PID constants */
  public static PIDController LINEAR_PID_GAINS = new PIDController(1.5, 0.002, 0);
  /** angular PID constants */
  public static ProfiledPIDController ANGULAR_PID_GAINS =
      new ProfiledPIDController(2, 0, 0, ANGLE_AUTO_CONSTRAINTS);
  /** the minimum distance from the current pose to the goal where we will not scale the velocity */
  public static double MIN_DISTANCE_VELOCITY_CORRECTION = 0.02;
  /** the minimum velocity the feed forward can get */
  public static double MIN_SET_POINT_VELOCITY = 0.5;
  /** the distance which above the FF is at 100% and below is 100%-0%. */
  public static double FF_MAX_DISTANCE = 0.1;
  /** the distance which above the FF is at 0%-100% and below is 0%. */
  public static double FF_MIN_DISTANCE = 0.025;
  /** the angle which above the FF is at 100% and below is 100%-0%. */
  public static double FF_MAX_ANGLE = Units.degreesToRadians(15);
  /** the angle which above the FF is at 0%-100% and below is 0%. */
  public static double FF_MIN_ANGLE = Units.degreesToRadians(5);
  /** the distance within the robot is considered at the goal */
  public static double POSE_TOLERANCE = 0.03; // m
  /** the angle within the robot is considered at the goal */
  public static double ANGLE_TOLERANCE = Units.degreesToRadians(2);
  /**
   * the velocity below the robot is considered stopped. if above the velocity the robot will not be
   * considered inside the tolerance.
   */
  public static double VELOCITY_TOLERANCE = 0.1; // m/s
  /**
   * the angular velocity below the robot is considered stopped. if above the angular velocity the
   * robot will not be considered inside the tolerance.
   */
  public static double ANGULAR_VELOCITY_TOLERANCE = Units.degreesToRadians(5); // rad/s
}
