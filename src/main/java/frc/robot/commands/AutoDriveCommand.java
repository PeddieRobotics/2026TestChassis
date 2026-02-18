// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveCommand extends Command {
    private Drivetrain drivetrain;
    private Command pathCommandBlue, pathCommandRed, pathCommand;

    List<Waypoint> generateWaypointsBlue(List<Pose2d> poses) {
        return PathPlannerPath.waypointsFromPoses(poses);
    }

    List<Waypoint> generateWaypointsRed(List<Pose2d> poses) {
        List<Pose2d> flippedPoses = new ArrayList<>();
        for (Pose2d pose : poses) {
            Pose2d newPose = new Pose2d(
                16.54 - pose.getX(),
                8.07 - pose.getY(),
                pose.getRotation().plus(Rotation2d.fromDegrees(180))
            );
            flippedPoses.add(newPose);
        }

        return PathPlannerPath.waypointsFromPoses(flippedPoses);
    }

    IdealStartingState getIdealStartingStateBlue(IdealStartingState start) {
        return start;
    }

    IdealStartingState getIdealStartingStateRed(IdealStartingState start) {
        Rotation2d rot = start.rotation();
        return new IdealStartingState(start.velocityMPS(), rot.plus(Rotation2d.fromDegrees(180)));
    }

    GoalEndState getGoalEndStateBlue(GoalEndState end) {
        return end;
    }

    GoalEndState getGoalEndStateRed(GoalEndState end) {
        Rotation2d rot = end.rotation();
        return new GoalEndState(end.velocityMPS(), rot.plus(Rotation2d.fromDegrees(180)));
    }
        
    List<RotationTarget> getHolonomicRotationsBlue(List<RotationTarget> holonomicRotations) {
        return holonomicRotations;
    }

    List<RotationTarget> getHolonomicRotationsRed(List<RotationTarget> holonomicRotations){
        List<RotationTarget> flippedRotations = new ArrayList<>();
        for (RotationTarget holonomicRotation : holonomicRotations) {
            Rotation2d newRotation = holonomicRotation.rotation().plus(Rotation2d.fromDegrees(180));
            flippedRotations.add(new RotationTarget(holonomicRotation.position(), newRotation));
        }
        return flippedRotations;
    }

    private void generatePaths(
        List<Pose2d> poseList,
        List<RotationTarget> holonomicRotations,
        List<ConstraintsZone> constraintZones,
        List<EventMarker> eventMarkers,
        PathConstraints constraints,
        IdealStartingState start,
        GoalEndState end
    ) {
        drivetrain = Drivetrain.getInstance();

        List<Waypoint> waypointsBlue = generateWaypointsBlue(poseList);
        List<Waypoint> waypointsRed = generateWaypointsRed(poseList);

        List<RotationTarget> holonomicRotationsBlue, holonomicRotationsRed;
        if (holonomicRotations == null)
            holonomicRotationsBlue = holonomicRotationsRed = Collections.emptyList();
        else {
            holonomicRotationsBlue = getHolonomicRotationsBlue(holonomicRotations);
            holonomicRotationsRed = getHolonomicRotationsRed(holonomicRotations);
        }

        if (constraintZones == null)
            constraintZones = Collections.emptyList();
        if (eventMarkers == null)
            eventMarkers = Collections.emptyList();

        IdealStartingState startBlue = getIdealStartingStateBlue(start);
        IdealStartingState startRed = getIdealStartingStateRed(start);

        GoalEndState endBlue = getGoalEndStateBlue(end);
        GoalEndState endRed = getGoalEndStateRed(end);

        PathPlannerPath pathBlue = new PathPlannerPath(
            waypointsBlue,
            holonomicRotationsBlue,
            Collections.emptyList(),
            constraintZones,
            eventMarkers,
            constraints,
            startBlue,
            endBlue,
            false
        );
        PathPlannerPath pathRed = new PathPlannerPath(
            waypointsRed,
            holonomicRotationsRed,
            Collections.emptyList(),
            constraintZones,
            eventMarkers,
            constraints,
            startRed,
            endRed,
            false
        );

        pathBlue.preventFlipping = pathRed.preventFlipping = true;

        pathCommandBlue = AutoBuilder.followPath(pathBlue);
        pathCommandRed = AutoBuilder.followPath(pathRed);
    }

    public AutoDriveCommand(
        List<Pose2d> poseList,
        PathConstraints constraints,
        IdealStartingState start,
        GoalEndState end) {
        drivetrain = Drivetrain.getInstance();
        generatePaths(poseList, null, null, null, constraints, start, end);
        addRequirements(drivetrain);
    }

    public AutoDriveCommand(
        List<Pose2d> poseList,
        List<EventMarker> eventMarkers,
        PathConstraints constraints,
        IdealStartingState start,
        GoalEndState end
    ) {
        drivetrain = Drivetrain.getInstance();
        generatePaths(poseList, null, null, eventMarkers, constraints, start, end);
        addRequirements(drivetrain);
    }


    public AutoDriveCommand(
        List<Pose2d> poseList,
        List<RotationTarget> holonomicRotations,
        List<EventMarker> eventMarkers,
        PathConstraints constraints,
        IdealStartingState start,
        GoalEndState end
    ) {
        drivetrain = Drivetrain.getInstance();
        generatePaths(poseList, holonomicRotations, null, eventMarkers, constraints, start, end);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pathCommand = (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue) ? pathCommandBlue : pathCommandRed;
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}
