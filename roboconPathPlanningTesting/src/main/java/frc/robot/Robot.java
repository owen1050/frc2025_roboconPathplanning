// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  public Robot() {

    // this line updates path planner to know the size of the field so when you call
    // flipPath it flips along the robocon midfield rather than the default midfield
    FlippingUtil.fieldSizeX = 15.68;
    testFlippingPath();
    importRoboConAprilTagLayout();

  }

  //this imports a path and flips it. both paths are converted to trajectories and put on SD to be viewed on advantage scope to verify the flip worked on the robocon field
  public void testFlippingPath() {
    Field2d originaField2d = new Field2d();
    Field2d mirrorField2d = new Field2d();

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("ExamplePath");
      PathPlannerTrajectory originalTraj = path.generateTrajectory(new ChassisSpeeds(), new Pose2d().getRotation(),
          RobotConfig.fromGUISettings());
      originaField2d.getObject("traj").setTrajectory(ppTrajToWPITraj(originalTraj));

      PathPlannerPath mirroredPath = path.flipPath();
      PathPlannerTrajectory mirTraj = mirroredPath.generateTrajectory(new ChassisSpeeds(), new Pose2d().getRotation(),
          RobotConfig.fromGUISettings());
      mirrorField2d.getObject("traj").setTrajectory(ppTrajToWPITraj(mirTraj));

      SmartDashboard.putData("Original", originaField2d);
      SmartDashboard.putData("Mirrored", mirrorField2d);

    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  //this imports the april tag layout for robocon and then puts all the poses of april tags on the dashboard to verify it matches new field map
  public void importRoboConAprilTagLayout() {
    String path = Filesystem.getDeployDirectory().getAbsolutePath() + "\\2025-reefscape-welded-robocon.json";
    try {
      AprilTagFieldLayout atflRobocon = new AprilTagFieldLayout(path);
                                                                      
      for (int i = 1; i <= 22; i++) {
        Field2d aprilTag = new Field2d();
        aprilTag.setRobotPose(atflRobocon.getTagPose(i).get().toPose2d());
        SmartDashboard.putData("Tag" + Integer.toString(i), aprilTag);
      }

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public Trajectory ppTrajToWPITraj(PathPlannerTrajectory traj) {
    List<PathPlannerTrajectoryState> stateList = traj.getStates();
    List<Trajectory.State> wpiStateLists = new ArrayList<Trajectory.State>();
    for (PathPlannerTrajectoryState state : stateList) {
      Trajectory.State thisWPIState = new Trajectory.State(state.timeSeconds,
          state.linearVelocity,
          0,
          state.pose,
          0);
      wpiStateLists.add(thisWPIState);
    }
    return new Trajectory(wpiStateLists);
  }
}
