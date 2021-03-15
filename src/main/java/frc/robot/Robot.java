/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.MPController;
import frc.robot.auto.Trajectories;

public class Robot extends TimedRobot {

  // Creates our Motion Profile Controller and Trajectories class
  Trajectories trajectories = new Trajectories();
  MPController mpController;
  Command autoCommand; 

  Trajectory testTraj;

  Trajectory.State goalPoint;

  ChassisSpeeds trajSpeeds;

  double trajectoryTime;

  @Override
  public void robotInit() {
    mpController = new MPController();

    testTraj = trajectories.generateAutoTrajectoryFromCurrentPose(1, new Pose2d());
    mpController.drive.setupMotorConfigs();

  }

  @Override
  public void robotPeriodic() {
    mpController.drive.putEncoder();
    mpController.drive.putGyro();
    SmartDashboard.putNumber("X Pose (Side-to-Side): ", mpController.drive.getPose().getX());
    SmartDashboard.putNumber("Y Pose (Forward-Back): ", mpController.drive.getPose().getY());
    SmartDashboard.putNumber("Rotation Pose: ", mpController.drive.getPose().getRotation().getDegrees());
    mpController.drive.putWheelVelocities();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {

    trajectoryTime = 0.0;
    // Initialize Auto Drive System

    // Reset encoders
    mpController.drive.resetEncoders();

    // Initialize our odometry
    mpController.drive.initializeOdometry();

    // Ensure our odometry is at 0
    mpController.drive.reset();

    // Update our odometry
    mpController.drive.periodic();    
  
    mpController.createMecanumFollowerCommand(testTraj, Units.metersToFeet(1.5)).schedule();
  }

  @Override
  public void autonomousPeriodic() {
    // Continue updating odometry while we use it in autonomous
      mpController.drive.periodic(); 

    //mpController.drive.setOutputVolts(new MecanumDriveMotorVoltages(3, 3, 3, 3));

    // goalPoint = testTraj.sample(trajectoryTime);

    // trajSpeeds = mpController.drive.HDC.calculate(mpController.drive.getPose(), goalPoint, goalPoint.poseMeters.getRotation());

    // mpController.drive.setOutputVelocity(mpController.drive.getKinematics().toWheelSpeeds(trajSpeeds));

    // trajectoryTime += 0.02;
    // SmartDashboard.putNumber("FL Expected:", mpController.drive.getKinematics().toWheelSpeeds(trajSpeeds).frontLeftMetersPerSecond);
    // SmartDashboard.putNumber("FR Expected: ", mpController.drive.getKinematics().toWheelSpeeds(trajSpeeds).frontRightMetersPerSecond);
    // SmartDashboard.putNumber("RL Expected: ", mpController.drive.getKinematics().toWheelSpeeds(trajSpeeds).rearLeftMetersPerSecond);
    // SmartDashboard.putNumber("RR Expected: ", mpController.drive.getKinematics().toWheelSpeeds(trajSpeeds).rearRightMetersPerSecond);
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
    // Cancel any commands that were running
    mpController.drive.setOutputVelocity(new MecanumDriveWheelSpeeds());
    CommandScheduler.getInstance().cancelAll();
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
}
