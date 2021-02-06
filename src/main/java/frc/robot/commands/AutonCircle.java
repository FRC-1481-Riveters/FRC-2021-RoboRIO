/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.*;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj2.command.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonCircle extends SequentialCommandGroup {

  
  public AutonCircle(Shooter shooter, Indexer indexer, Kicker kicker,
  DriveTrain drive, Intake intake)
  {

          // Create a voltage constraint to ensure we don't accelerate too fast
          var autoVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(Constants.ksVolts,
                                         Constants.kvVoltSecondsPerMeter,
                                         Constants.kaVoltSecondsSquaredPerMeter),
              Constants.kDriveKinematics,
              4);
        
        // Create config for trajectory
        TrajectoryConfig config =
          new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                               Constants.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(Constants.kDriveKinematics)
              // Apply the voltage constraint
              .addConstraint(autoVoltageConstraint);
        
        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(
              new Translation2d(1, 1),
              new Translation2d(2, -1)
          ),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          // Pass config
          config
        );
        
        RamseteCommand ramseteCommand = new RamseteCommand(
          exampleTrajectory,
          drive::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.ksVolts,
                                     Constants.kvVoltSecondsPerMeter,
                                     Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drive::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drive::tankDriveVolts,
          drive
        );
        
        
  drive.resetOdometry(exampleTrajectory.getInitialPose());
    
    // Run path following command, then stop at the end.
  addCommands( 
    ramseteCommand,
    new AutonRobotDriveDistance(drive, 0.0) // stop the robot
  );
}
}