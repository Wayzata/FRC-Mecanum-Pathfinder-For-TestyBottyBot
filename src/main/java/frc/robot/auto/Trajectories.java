package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

import java.awt.*;
import java.util.Arrays;

public class Trajectories {

    /**
     * Trajectory Configs 
     */
    TrajectoryConfig testConfig = new TrajectoryConfig(0.5, 0.5);

    public Trajectories() {
        testConfig.setStartVelocity(0);
        testConfig.setEndVelocity(0);
    }

    /**
     * 
     * @param trajectoryID - Available: [1 - Test Trajectory] [2 - Trench Pickup from Center]
     * @param currentPose - The current position(x, y, z) of the robot in the object Pose2d
     * @return
     */
    public Trajectory generateAutoTrajectoryFromCurrentPose(int trajectoryID, Pose2d currentPose){
        switch (trajectoryID){
            case 1: //Blue line to blue trench
                return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(Units.feetToMeters(8), Units.feetToMeters(-6), Rotation2d.fromDegrees(180))),
                    testConfig
                );
            case 2: //Red line to red trench
                return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(Units.feetToMeters(-8), Units.feetToMeters(6), Rotation2d.fromDegrees(180))),
                    testConfig
                );
            case 3:
            return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(currentPose, new Pose2d(0, Units.feetToMeters(5), Rotation2d.fromDegrees(0))), // Makes the robot go 5 feet in y direction on autonomousInit();
                testConfig
            );

            default:
                return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(currentPose, new Pose2d(0, Units.feetToMeters(5), Rotation2d.fromDegrees(0))), // Makes the robot go 5 feet in y direction on autonomousInit();
                    testConfig
                );
        }
    }

    /**
     * Below is used only to help visually see the field and different lengths. Run this main on the computer 
     */
    public static void main(String[] args) {
        double[][] base = new double[][]{{0,0}};
        FalconLinePlot fig = new FalconLinePlot(base, Color.blue, Color.blue);
        fig.setXTic(0, 52, 1);
        fig.setYTic(0, 27, 1);
        fig.xGridOn();
        fig.yGridOn();
        drawInfiniteRecharge(fig);
    }

    public static void drawInfiniteRecharge(FalconLinePlot fig){
        double[][] autoLineRed = new double[][] {{10,0}, {10, 27}};
        double[][] autoLineBlue = new double[][] {{42, 0}, {42, 27}};
        double[][] blueTrench = new double[][] {{17, 0}, {17, 4.5}, {35, 4.5}, {35,0}};
        double[][] redTrench = new double[][] {{17, 27}, {17, 22.5}, {35, 22.5}, {35,27}};
        double[][] redTargetZone = new double[][] {{0, 17}, {2.5, 19}, {0, 21}};
        double[][] blueTargetZone = new double[][] {{52, 10}, {49.5, 8}, {52, 6}};
        double[][] climbTower = new double[][] {{17, 17}, {22, 4.5}, {35, 10}, {30, 22.5}, {17, 17}};
        double[][] redControlPanel = new double[][] {{28.5, 27}, {28.5, 22.5}, {31, 22.5}, {31, 27}};
        double[][] blueControlPanel = new double[][] {{23.5, 0}, {23.5, 4.5}, {21, 4.5}, {21, 0}};
    
        fig.addData(autoLineBlue, Color.black);
        fig.addData(autoLineRed, Color.black);
        fig.addData(blueTrench, Color.blue);
        fig.addData(redTrench, Color.red);
        fig.addData(redTargetZone, Color.red);
        fig.addData(blueTargetZone, Color.blue);
        fig.addData(climbTower, Color.black);
        fig.addData(redControlPanel, Color.black);
        fig.addData(blueControlPanel, Color.black);
        drawBall(fig, 20, 24.75);
        drawBall(fig, 23, 24.75);
        drawBall(fig, 26, 24.75);
        drawBall(fig, 31.25, 25.5);
        drawBall(fig, 31.25, 24);
      }
    
      private static void drawBall(FalconLinePlot fig, double x, double y){
        final int NUM_POINTS = 15;
        final double RADIUS = 0.58;
        
        double[] ballCoordinateX = new double[16];
        double[] ballCoordinateY = new double[16];
    
        for (int i = 0; i <= NUM_POINTS; ++i)
        {
            final double angle = Math.toRadians(((double) i / NUM_POINTS) * 360d);
            ballCoordinateX[i] = Math.cos(angle) * RADIUS + x;
            ballCoordinateY[i] = Math.sin(angle) * RADIUS + y; 
        }
        fig.addData(ballCoordinateX, ballCoordinateY, Color.yellow);
    
      }
}