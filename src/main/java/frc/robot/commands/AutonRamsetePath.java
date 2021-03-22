// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.DriverStation;

public class AutonRamsetePath extends RamseteCommand {

  private static Trajectory trajectory;
  private static DriveTrain driveTrain;
  private static boolean stopAfter;

  public AutonRamsetePath( DriveTrain m_drive, Trajectory m_trajectory, boolean m_stopAfter )
    {
      super( m_trajectory,
            m_drive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                    Constants.kvVoltSecondsPerMeter,
                                    Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            m_drive::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drive::tankDriveVolts,
            m_drive );

    driveTrain = m_drive;
    trajectory = m_trajectory;
    stopAfter = m_stopAfter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetOdometry( trajectory.getInitialPose() );
    DriverStation.reportError("Robot X set to " + driveTrain.m_odometry.getPoseMeters().getX() +  "  Robot Y set to " +  driveTrain.m_odometry.getPoseMeters().getY(), false);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end( interrupted );
    if( stopAfter ) 
      driveTrain.tankDriveVolts(0, 0);
  }

}