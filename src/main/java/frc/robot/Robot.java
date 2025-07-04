// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.json.JSONObject;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

    addPeriodic(()->m_robotContainer.locengine.sendState(), 0.40);
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
    String s = "%s  %s";
    // System.out.println(String.format(s, 
    // m_robotContainer.locengine.procSideLeft(), 
    // m_robotContainer.locengine.reefSideLeft(), 
    // m_robotContainer.locengine.bargeSideLeft()));

    Optional<EstimatedRobotPose> pose = m_robotContainer.downCamera.getEstimatedGlobalPose(m_robotContainer.drivetrain.getPos());
    if (pose.isPresent()) {
      System.out.println(String.format(s,pose.get().estimatedPose.toPose2d(),pose.get()) );
    }
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
