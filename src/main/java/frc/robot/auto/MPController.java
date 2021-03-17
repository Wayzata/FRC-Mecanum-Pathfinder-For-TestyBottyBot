package frc.robot.auto;

import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;

public class MPController {

    public AutoDriveSystem drive = new AutoDriveSystem();

    /**
     * Creates a new trajectory follower command for a mecanum drivetrain
     * @param trajectory - The desired trajectory you want the command to follow.
     * @param trajectoryMaxSpeed - the max speed of the trajectory you supplied in m/s.
     * @return MecanumControllerCommand that can be scheduled to run.
     */
    public Command createMecanumFollowerCommand(Trajectory trajectory, double trajectoryMaxSpeed) {
        MecanumControllerCommand command = new MecanumControllerCommand(
            trajectory,
            drive::getPose,
            drive.getFeedforward(),
            drive.getKinematics(),
            drive.getXController(),
            drive.getYController(),
            drive.getThetaController(),
            trajectoryMaxSpeed,
            drive.getFrontLeftPIDController(),
            drive.getBackLeftPIDController(),
            drive.getFrontRightPIDController(),
            drive.getBackRightPidController(),
            drive::getSpeeds,
            drive::setOutputVolts
        );
        return command.andThen(() -> drive.setOutputVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0)));
         // .andThen(() -> drive.setOutputVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0)))
      }  
      
      /**
     * Creates a new trajectory follower command for a mecanum drivetrain
     * @param trajectory - The desired trajectory you want the command to follow.
     * @param maxSpeedFt - the max speed of the trajectory you supplied in ft/s.
     * @return MecanumControllerCommand that can be scheduled to run.
     */
    public Command createMecanumVelocityCommand(Trajectory trajectory, double trajectoryMaxSpeedFt) {
        MecanumControllerCommand command = new MecanumControllerCommand(
            trajectory,
            drive::getPose,
            drive.getKinematics(),
            drive.getXController(),
            drive.getYController(),
            drive.getThetaController(),
            Units.feetToMeters(trajectoryMaxSpeedFt),
            drive::setOutputVelocity
        );
        return command.andThen(() -> drive.setOutputVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0)));
         // .andThen(() -> drive.setOutputVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0)))
    }
}