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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    FlippingUtil.fieldSizeX = 15.8;

    Field2d originaField2d = new Field2d();
    Field2d mirrorField2d = new Field2d();

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("ExamplePath");
      PathPlannerTrajectory originalTraj = path.generateTrajectory(new ChassisSpeeds(), new Pose2d().getRotation(), RobotConfig.fromGUISettings());
      originaField2d.getObject("traj").setTrajectory(ppTrajToWPITraj(originalTraj));

      PathPlannerPath mirroredPath = path.flipPath();
      PathPlannerTrajectory mirTraj = mirroredPath.generateTrajectory(new ChassisSpeeds(), new Pose2d().getRotation(), RobotConfig.fromGUISettings());
      mirrorField2d.getObject("traj").setTrajectory(ppTrajToWPITraj(mirTraj));

      SmartDashboard.putData("Original", originaField2d);
      SmartDashboard.putData("Mirrored", mirrorField2d);

    } catch (FileVersionException | IOException | ParseException e) {
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
