package com.pathplanner.lib.path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
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

public class DriveToPose extends Command {
    private static DriveToPoseConstants constants;
    private static boolean configured = false;

    public static void configure(DriveToPoseConstants driveToPoseConstants) {
        if (configured) {
            throw new IllegalStateException("Already configured");
        }
        constants = driveToPoseConstants;
        configured = true;

    }

    /**
     * the goal state in the Trapezoid Profile
     */
    private final static TrapezoidProfile.State GOAL_STATE = new TrapezoidProfile.State(0,0);
    /**
     * the linear Trapezoid Profile
     */
    private TrapezoidProfile driveProfile;
    /**
     * the position in the last run
     */
    private Translation2d lastSetPoint;
    /**
     * the velocity in the last run
     */
    private Vector<N2> lastSetpointVelocity;
    /**
     * absolute value of the angle error
     */
    private double absAngleError;
    /**
     * absolute value of the linear error
     */
    private double absPoseError;

    private final Pose2d goalPose_;


    public DriveToPose(Pose2d goalPose) {
        if(!configured) {throw new IllegalStateException("Not configured!");}

        this.goalPose_ = goalPose;
        
        constants.ANGULAR_PID_GAINS.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Command createPathToPose(PathPlannerPath path) {
        Command pathCommand = AutoBuilder.followPath(path);

        if(!constants.useDriveToPose) return pathCommand;

        SequentialCommandGroup sequence = new SequentialCommandGroup();

        var finalPose = path.getPoint(path.numPoints() - 1);

        BooleanSupplier isClose = () -> constants.poseSupplier.get().getTranslation().getDistance(finalPose.position) <
                constants.DISTANCE_TO_STOP_PP;

        sequence.addCommands(pathCommand.until(isClose),
                new DriveToPose(new Pose2d(finalPose.position, finalPose.rotationTarget.rotation())));

        return sequence;
    }


    /**
     * rests all the variables before starting
     */
    private void resetState(){
        driveProfile = new TrapezoidProfile(RobotState.isAutonomous() ?
                constants.LINEAR_AUTO_CONSTRAINTS :
                constants.LINEAR_TELE_CONSTRAINTS);

        constants.LINEAR_PID_GAINS.reset();

        constants.ANGULAR_PID_GAINS.setConstraints(RobotState.isAutonomous() ?
                constants.ANGLE_AUTO_CONSTRAINTS:
                constants.ANGLE_TELE_CONSTRAINTS);

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(constants.speedsSupplier.get(), constants.poseSupplier.get().getRotation());

        constants.ANGULAR_PID_GAINS.reset(new TrapezoidProfile.State(constants.poseSupplier.get().getRotation().getRadians(), speeds.omegaRadiansPerSecond));

        lastSetPoint = constants.poseSupplier.get().getTranslation();

        lastSetpointVelocity = VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        var error = constants.poseSupplier.get().relativeTo(goalPose_);

        absAngleError = Math.abs(error.getRotation().getRadians());

        absPoseError = error.getTranslation().getNorm();
    }
    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        resetState();
        constants.logging.accept("driveToPose/goalPose", goalPose_);
    }

    /**
     * calculates the next state in the profile.
     * @param goal final goal
     * @param pose current pose
     * @param error the distance between the current pose and the goal.
     * @return the next state of the profile
     */
    public TrapezoidProfile.State calcProfile(Pose2d goal, Pose2d pose, double error) {
        var profileDirection2d = goal.getTranslation().minus(lastSetPoint);
        var profileDirection = profileDirection2d.toVector();

        //if the distance from the current pose to the goal is less then a constant then we will not scale the velocity
        //vector according to the direction vector.
        double velocity = profileDirection.norm()
                <= constants.MIN_DISTANCE_VELOCITY_CORRECTION
                ? lastSetpointVelocity.norm()
                : lastSetpointVelocity.dot(profileDirection) / profileDirection.norm();

        velocity = Math.max(velocity, constants.MIN_SET_POINT_VELOCITY);

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(profileDirection.norm(), -velocity);

        var nextState = driveProfile.calculate(0.02, currentState, GOAL_STATE);

        double scalar = nextState.position / error;

        //let v be current pose vector
        //let u be goal pose vector
        //vectoric function:
        //w = v * t + u * (1 - t)
        //if you are reading this use Translation2D.interpolate instead
        lastSetPoint = pose.getTranslation().times(scalar).plus(goal.getTranslation().times(1 - scalar));
        return nextState;
    }

    /**
     * calc the FFScalar for the velocity
     * @param error the distance between the current pose and the goal.
     * @return how much should the velocity FF affect the output.
     */
    public double FFScalar(double error, double minError, double maxError) {
        return MathUtil.clamp((error - minError)/ (maxError - minError),
                0.0, 1.0);
    }

    @Override
    public void execute() {
        Pose2d pose = constants.poseSupplier.get();
        Pose2d goal = goalPose_;
        var poseError = pose.relativeTo(goal);
        absPoseError = poseError.getTranslation().getNorm();
        absAngleError = Math.abs(poseError.getRotation().getRadians());
        var nextState = calcProfile(goal, pose, absPoseError);

        double targetVelocityPID = constants.LINEAR_PID_GAINS.calculate(absPoseError, nextState.position);

        double targetVelocityFF = nextState.velocity * FFScalar(absPoseError, constants.FF_MIN_DISTANCE,
                constants.FF_MAX_DISTANCE);

        var errorAngle = pose.getTranslation().minus(goal.getTranslation()).getAngle();

        lastSetpointVelocity = new Translation2d(targetVelocityFF, errorAngle).toVector();

        double targetVelocity = targetVelocityPID + targetVelocityFF;


        if(absPoseError <= constants.POSE_TOLERANCE)
            targetVelocity = 0;

        targetVelocity = Math.signum(targetVelocity) * Math.min(Math.abs(targetVelocity),
                constants.MAX_LINEAR_SPEED);

        var driveVelocity = new Translation2d(targetVelocity, errorAngle);

        double angularVelocityPID = constants.ANGULAR_PID_GAINS.calculate(pose.getRotation().getRadians(),
                goal.getRotation().getRadians());

        double angularVelocityFF = constants.ANGULAR_PID_GAINS.getSetpoint().velocity * FFScalar(absAngleError,
                constants.FF_MIN_ANGLE, constants.FF_MAX_ANGLE);

        double angularVelocity = angularVelocityPID + angularVelocityFF;


        if(absAngleError <= constants.ANGLE_TOLERANCE)
            angularVelocity = 0;

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(), driveVelocity.getY(), angularVelocity,
                pose.getRotation());

        constants.drive.accept(speeds);

        //Logging
        constants.logging.accept("driveToPose/lastSetPoint", new Pose2d(lastSetPoint,
                Rotation2d.fromRadians(constants.ANGULAR_PID_GAINS.getSetpoint().position)));

        constants.logging.accept("driveToPose/nextState/position", nextState.position);
        constants.logging.accept("driveToPose/nextState/velocity", nextState.velocity);

        constants.logging.accept("driveToPose/angularError", absAngleError);
        constants.logging.accept("driveToPose/poseError", absPoseError);

        constants.logging.accept("driveToPose/velocity/total", targetVelocity);
        constants.logging.accept("driveToPose/velocity/PID", targetVelocityPID);
        constants.logging.accept("driveToPose/velocity/feedForward", targetVelocityFF);

        constants.logging.accept("driveToPose/angularVelocity/total", angularVelocity);
        constants.logging.accept("driveToPose/angularVelocity/PID", angularVelocityPID);
        constants.logging.accept("driveToPose/angularVelocity/feedForward", angularVelocityFF);



    }

    @Override
    public boolean isFinished() {
        var speeds = constants.speedsSupplier.get();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);
        return absAngleError <= constants.ANGLE_TOLERANCE
                && absPoseError <= constants.POSE_TOLERANCE
                && linearSpeed <= constants.VELOCITY_TOLERANCE
                && angularSpeed <= constants.ANGULAR_VELOCITY_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        constants.drive.accept(new ChassisSpeeds());
    }

}
