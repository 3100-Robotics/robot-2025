// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.json.JSONObject;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LocationsFake;
import frc.robot.math.LocatorEngine;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  JSONObject jsonObject = new JSONObject();

  public Robot() {
    m_robotContainer = new RobotContainer();

    addPeriodic(() -> {
      Optional<EstimatedRobotPose> pose = m_robotContainer.downCamera.getEstimatedGlobalPose(m_robotContainer.drivetrain.getPos());
      if (pose.isPresent()) {
        m_robotContainer.drivetrain.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
      }}, 0.020); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // String s = "%s  %s  %s";
    // System.out.println(String.format(s, 
    // m_robotContainer.locengine.procSideLeft(), 
    // m_robotContainer.locengine.reefSideLeft(), 
    // m_robotContainer.locengine.bargeSideLeft()));

    LocatorEngine le = m_robotContainer.locengine;

    double rcord[] = {le.getRobotNormalized.get().x, le.getRobotNormalized.get().y};
    double vec[] = {Math.cos(Math.toRadians(le.getRobotRotation360.get())), Math.sin(Math.toRadians(le.getRobotRotation360.get()))};
    double pointsr[][] = {rcord, vec};
    jsonObject.put("robot", pointsr);

    double down[] = {LocationsFake.REEF_DOWNLEFT.x, LocationsFake.REEF_DOWNLEFT.y};
    double left[] = {LocationsFake.REEF_LEFT.x, LocationsFake.REEF_LEFT.y};
    double up[] = {LocationsFake.REEF_UP.x, LocationsFake.REEF_UP.y};
    double right[] = {LocationsFake.REEF_RIGHT.x, LocationsFake.REEF_RIGHT.y};
    double reef_points[][] = {down,left,up,right};
    jsonObject.put("reef_points", reef_points);

    double seven[] = {LocationsFake.SEVEN.x, LocationsFake.SEVEN.y};
    double three[] = {LocationsFake.THREE.x, LocationsFake.THREE.y};
    double one[] = {LocationsFake.ONE.x, LocationsFake.ONE.y};
    double reef_sides[][] = {seven,three,one};
    jsonObject.put("reef_sides", reef_sides);

    SmartDashboard.putString("qdbpoints", jsonObject.toString());
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
