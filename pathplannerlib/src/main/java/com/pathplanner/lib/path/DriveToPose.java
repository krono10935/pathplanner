package com.pathplanner.lib.path;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drive to pose will finish a pathplanner path until the target pose is reached, it uses a vectored
 * PID controller instead of a xy controller. Any constants are stored inside {@link
 * DriveToPoseConstants}.
 */
public class DriveToPose extends Command {
  /** all the constants from the DriveToPoseConstants */
  private static DriveToPoseConstants constants;
  /** if configured */
  private static boolean configured = false;

  private static DriveToPose instance;

  /**
   * this function needs to be called before any paths are made.
   *
   * @param constants the DriveToPose constants.
   */
  public static void configure(DriveToPoseConstants constants) {
    if (configured) {
      throw new IllegalStateException("Already configured");
    }
    DriveToPose.constants = constants;
    configured = true;

    logEmpty();
  }

  /** the goal state in the Trapezoid Profile */
  private static final TrapezoidProfile.State GOAL_STATE = new TrapezoidProfile.State(0, 0);
  /** the linear Trapezoid Profile */
  private TrapezoidProfile driveProfile;
  /** the position in the last run */
  private Translation2d lastSetPoint;
  /** the velocity in the last run */
  private Vector<N2> lastSetpointVelocity;
  /** absolute value of the angle error */
  private double absAngleError;
  /** absolute value of the linear error */
  private double absPoseError;
  /** the goal position */
  private Pose2d goalPose;

  /** creates the DriveToPose with a given goal pose. */
  private DriveToPose() {
    if (!configured) {
      throw new IllegalStateException("Not configured!");
    }

    DriveToPoseConstants.ANGULAR_PID_GAINS.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * creates a command with a given path and the DriveToPose (if set to true)
   *
   * @param path the PathPlanner path
   * @return the full command where the PathPlanner is integrated with the DriveToPose (if set to
   *     true)
   */
  public static Command createPathToPose(PathPlannerPath path) {
    Command pathCommand = AutoBuilder.followPath(path);

    if (path.getGoalEndState().velocity().baseUnitMagnitude() != 0) return pathCommand;

    SequentialCommandGroup sequence = new SequentialCommandGroup();

    var finalPose = path.getPoint(path.numPoints() - 1);

    BooleanSupplier isClose =
        () ->
            constants.poseSupplier().get().getTranslation().getDistance(finalPose.position)
                < DriveToPoseConstants.DISTANCE_TO_STOP_PP;

    Pose2d newGoalPose = new Pose2d(finalPose.position, finalPose.rotationTarget.rotation());

    if (instance == null) {
      instance = new DriveToPose();
    }

    Command driveToPose = instance.asProxy().beforeStarting(() -> instance.goalPose = newGoalPose);

    sequence.addCommands(pathCommand.until(isClose), driveToPose);

    return sequence;
  }

  /** rests all the variables before starting */
  private void resetState() {
    driveProfile =
        new TrapezoidProfile(
            RobotState.isAutonomous()
                ? DriveToPoseConstants.LINEAR_AUTO_CONSTRAINTS
                : DriveToPoseConstants.LINEAR_TELE_CONSTRAINTS);

    DriveToPoseConstants.LINEAR_PID_GAINS.reset();

    DriveToPoseConstants.ANGULAR_PID_GAINS.setConstraints(
        RobotState.isAutonomous()
            ? DriveToPoseConstants.ANGLE_AUTO_CONSTRAINTS
            : DriveToPoseConstants.ANGLE_TELE_CONSTRAINTS);

    var speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            constants.speedsSupplier().get(), constants.poseSupplier().get().getRotation());

    DriveToPoseConstants.ANGULAR_PID_GAINS.reset(
        new TrapezoidProfile.State(
            constants.poseSupplier().get().getRotation().getRadians(),
            speeds.omegaRadiansPerSecond));

    lastSetPoint = constants.poseSupplier().get().getTranslation();

    lastSetpointVelocity = VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    var error = constants.poseSupplier().get().relativeTo(goalPose);

    absAngleError = Math.abs(error.getRotation().getRadians());

    absPoseError = error.getTranslation().getNorm();
  }

  /** log every log with empty value/0. */
  private static void logEmpty() {
    Logger.recordOutput("driveToPose/goalPose", new Pose2d());
    Logger.recordOutput("driveToPose/lastSetPoint", new Pose2d());

    Logger.recordOutput("driveToPose/nextState/position", 0);
    Logger.recordOutput("driveToPose/nextState/velocity", 0);

    Logger.recordOutput("driveToPose/angularError", 0);
    Logger.recordOutput("driveToPose/poseError", 0);

    Logger.recordOutput("driveToPose/velocity/total", 0);
    Logger.recordOutput("driveToPose/velocity/PID", 0);
    Logger.recordOutput("driveToPose/velocity/feedForward", 0);

    Logger.recordOutput("driveToPose/angularVelocity/total", 0);
    Logger.recordOutput("driveToPose/angularVelocity/PID", 0);
    Logger.recordOutput("driveToPose/angularVelocity/feedForward", 0);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    resetState();
    Logger.recordOutput("driveToPose/goalPose", goalPose);
  }

  /**
   * calculates the next state in the profile.
   *
   * @param goal final goal
   * @param pose current pose
   * @param error the distance between the current pose and the goal.
   * @return the next state of the profile
   */
  public TrapezoidProfile.State calcProfile(Pose2d goal, Pose2d pose, double error) {
    var profileDirection2d = goal.getTranslation().minus(lastSetPoint);
    var profileDirection = profileDirection2d.toVector();

    // if the distance from the current pose to the goal is less then a constant then we will not
    // scale the velocity
    // vector according to the direction vector.
    double velocity =
        profileDirection.norm() <= DriveToPoseConstants.MIN_DISTANCE_VELOCITY_CORRECTION
            ? lastSetpointVelocity.norm()
            : lastSetpointVelocity.dot(profileDirection) / profileDirection.norm();

    velocity = Math.max(velocity, DriveToPoseConstants.MIN_SET_POINT_VELOCITY);

    TrapezoidProfile.State currentState =
        new TrapezoidProfile.State(profileDirection.norm(), -velocity);

    var nextState = driveProfile.calculate(0.02, currentState, GOAL_STATE);

    double scalar = nextState.position / error;

    // let v be current pose vector
    // let u be goal pose vector
    // vectoric function:
    // w = v * t + u * (1 - t)
    // if you are reading this use Translation2D.interpolate instead
    lastSetPoint =
        pose.getTranslation().times(scalar).plus(goal.getTranslation().times(1 - scalar));
    return nextState;
  }

  /**
   * calc the FFScalar for the velocity
   *
   * @param error the distance between the current pose and the goal.
   * @param minError the minimum value which at and below the function will return 0
   * @param maxError the maximum value which at and above the function will return 1
   * @return how much should the velocity FF affect the output.
   */
  public double FFScalar(double error, double minError, double maxError) {
    return MathUtil.clamp((error - minError) / (maxError - minError), 0.0, 1.0);
  }

  @Override
  public void execute() {
    Pose2d pose = constants.poseSupplier().get();
    Pose2d goal = goalPose;
    var poseError = pose.relativeTo(goal);
    absPoseError = poseError.getTranslation().getNorm();
    absAngleError = Math.abs(poseError.getRotation().getRadians());
    var nextState = calcProfile(goal, pose, absPoseError);

    double targetVelocityPID =
        DriveToPoseConstants.LINEAR_PID_GAINS.calculate(absPoseError, nextState.position);

    double targetVelocityFF =
        nextState.velocity
            * FFScalar(
                absPoseError,
                DriveToPoseConstants.FF_MIN_DISTANCE,
                DriveToPoseConstants.FF_MAX_DISTANCE);

    var errorAngle = pose.getTranslation().minus(goal.getTranslation()).getAngle();

    lastSetpointVelocity = new Translation2d(targetVelocityFF, errorAngle).toVector();

    double targetVelocity = targetVelocityPID + targetVelocityFF;

    if (absPoseError <= DriveToPoseConstants.POSE_TOLERANCE) targetVelocity = 0;

    targetVelocity =
        Math.signum(targetVelocity)
            * Math.min(Math.abs(targetVelocity), DriveToPoseConstants.MAX_LINEAR_SPEED);

    var driveVelocity = new Translation2d(targetVelocity, errorAngle);

    double angularVelocityPID =
        DriveToPoseConstants.ANGULAR_PID_GAINS.calculate(
            pose.getRotation().getRadians(), goal.getRotation().getRadians());

    double angularVelocityFF =
        DriveToPoseConstants.ANGULAR_PID_GAINS.getSetpoint().velocity
            * FFScalar(
                absAngleError,
                DriveToPoseConstants.FF_MIN_ANGLE,
                DriveToPoseConstants.FF_MAX_ANGLE);

    double angularVelocity = angularVelocityPID + angularVelocityFF;

    if (absAngleError <= DriveToPoseConstants.ANGLE_TOLERANCE) angularVelocity = 0;

    var speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), angularVelocity, pose.getRotation());

    constants.drive().accept(speeds);

    // Logging
    Logger.recordOutput(
        "driveToPose/lastSetPoint",
        new Pose2d(
            lastSetPoint,
            Rotation2d.fromRadians(DriveToPoseConstants.ANGULAR_PID_GAINS.getSetpoint().position)));

    Logger.recordOutput("driveToPose/nextState/position", nextState.position);
    Logger.recordOutput("driveToPose/nextState/velocity", nextState.velocity);

    Logger.recordOutput("driveToPose/angularError", absAngleError);
    Logger.recordOutput("driveToPose/poseError", absPoseError);

    Logger.recordOutput("driveToPose/velocity/total", targetVelocity);
    Logger.recordOutput("driveToPose/velocity/PID", targetVelocityPID);
    Logger.recordOutput("driveToPose/velocity/feedForward", targetVelocityFF);

    Logger.recordOutput("driveToPose/angularVelocity/total", angularVelocity);
    Logger.recordOutput("driveToPose/angularVelocity/PID", angularVelocityPID);
    Logger.recordOutput("driveToPose/angularVelocity/feedForward", angularVelocityFF);
  }

  @Override
  public boolean isFinished() {
    var speeds = constants.speedsSupplier().get();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);
    return absAngleError <= DriveToPoseConstants.ANGLE_TOLERANCE
        && absPoseError <= DriveToPoseConstants.POSE_TOLERANCE
        && linearSpeed <= DriveToPoseConstants.VELOCITY_TOLERANCE
        && angularSpeed <= DriveToPoseConstants.ANGULAR_VELOCITY_TOLERANCE;
  }

  @Override
  public void end(boolean interrupted) {
    constants.drive().accept(new ChassisSpeeds());
  }
}
