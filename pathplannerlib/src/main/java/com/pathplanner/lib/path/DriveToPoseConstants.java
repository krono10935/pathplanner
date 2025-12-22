package com.pathplanner.lib.path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class DriveToPoseConstants {
    public final Supplier<Pose2d> poseSupplier;
    public final Supplier<ChassisSpeeds> speedsSupplier;
    public final Consumer<ChassisSpeeds> drive;
    public final BiConsumer<String, Object> logging;
    public final boolean useDriveToPose;

    public double MAX_LINEAR_SPEED = 4; //m/s
    /**
     * the distance in which the auto should transfer from PP path to DriveToPose.
     */
    public double DISTANCE_TO_STOP_PP = 0.751; //m
    /**
     * the linear Trapezoid Profile constraints for autonomous
     */
    public TrapezoidProfile.Constraints LINEAR_AUTO_CONSTRAINTS =
            new TrapezoidProfile.Constraints(4, 1);
    /**
     * the linear Trapezoid Profile constraints for tele-op
     */
    public TrapezoidProfile.Constraints LINEAR_TELE_CONSTRAINTS =
            new TrapezoidProfile.Constraints(4, 1);
    /**
     * the angle Trapezoid Profile constraints for autonomous
     */
    public TrapezoidProfile.Constraints ANGLE_AUTO_CONSTRAINTS =
            new TrapezoidProfile.Constraints(9.753 / 1.2, Units.degreesToRadians(45));
    /**
     * the angle Trapezoid Profile constraints for tele-op
     */
    public TrapezoidProfile.Constraints ANGLE_TELE_CONSTRAINTS =
            new TrapezoidProfile.Constraints(9.753 / 1.2, Units.degreesToRadians(45));
    /**
     * linear PID constants
     */
    public PIDController LINEAR_PID_GAINS = new PIDController(1.5, 0.002, 0);
    /**
     * angular PID constants
     */
    public ProfiledPIDController ANGULAR_PID_GAINS = new ProfiledPIDController(2, 0, 0, ANGLE_AUTO_CONSTRAINTS);
    /**
     * the minimum distance from the current pose to the goal where we will not scale the velocity
     */
    public double MIN_DISTANCE_VELOCITY_CORRECTION = 0.02;
    /**
     * the minimum velocity the feed forward can get
     */
    public double MIN_SET_POINT_VELOCITY = 0.5;
    /**
     * the distance which above the FF is at 100% and below is 100%-0%.
     */
    public double FF_MAX_DISTANCE = 0.1;
    /**
     * the distance which above the FF is at 0%-100% and below is 0%.
     */
    public double FF_MIN_DISTANCE = 0.025;
    /**
     * the angle which above the FF is at 100% and below is 100%-0%.
     */
    public double FF_MAX_ANGLE = Units.degreesToRadians(15);
    /**
     * the angle which above the FF is at 0%-100% and below is 0%.
     */
    public double FF_MIN_ANGLE = Units.degreesToRadians(5);
    /**
     * the distance within the robot is considered at the goal
     */
    public double POSE_TOLERANCE = 0.03; //m
    /**
     * the angle within the robot is considered at the goal
     */
    public double ANGLE_TOLERANCE = Units.degreesToRadians(2);
    /**
     * the velocity below the robot is considered stopped. if above the velocity the robot will not be considered
     * inside the tolerance.
     */
    public double VELOCITY_TOLERANCE = 0.1; //m/s
    /**
     * the angular velocity below the robot is considered stopped. if above the angular velocity the robot will not
     * be considered inside the tolerance.
     */
    public double ANGULAR_VELOCITY_TOLERANCE = Units.degreesToRadians(5); //rad/s

    public DriveToPoseConstants(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier, Consumer<ChassisSpeeds> drive, BiConsumer<String, Object> logging, boolean useDriveToPose) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.drive = drive;
        this.logging = logging;
        this.useDriveToPose = useDriveToPose;
    }
}
