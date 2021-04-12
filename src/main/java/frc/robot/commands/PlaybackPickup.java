// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class PlaybackPickup extends CommandBase {
    DriveTrain m_drivetrain;
    Intake m_intake;
    Indexer  m_indexer;
  /** Creates a new PlaybackPickup. */
  public PlaybackPickup(DriveTrain drivetrain, Intake intake, Indexer indexer) {
        m_drivetrain = drivetrain;
        m_intake  = intake;
        m_indexer = indexer;
        addRequirements(drivetrain, intake, indexer);
//        new ParallelCommandGroup( //
      new AutonMacroPlayback( "/home/lvuser/autonpath.csv", m_drivetrain, m_intake, m_indexer ); /*, //
      new SequentialCommandGroup( //
        new IntakePositionPowerCellCommand(m_intake), // Pull in a PowerCell with the intake
        new ParallelCommandGroup( //
          new IntakeRunForABit(m_intake, .75), // Pin the Power Cell against the indexer
          new IndexerStackOnePowerCell(m_indexer) //
            .withTimeout(Constants.indexerStack1PwrCellTimeout)
        ),
        new ParallelCommandGroup( //
          new IntakeRunForABit(m_intake, .75), // Pin the Power Cell against the indexer
          new IndexerStackOnePowerCell(m_indexer) //
            .withTimeout(Constants.indexerStack1PwrCellTimeout)
        ),
        new ParallelCommandGroup( //
          new IntakeRunForABit(m_intake, .75), // Pin the Power Cell against the indexer
          new IndexerStackOnePowerCell(m_indexer) //
            .withTimeout(Constants.indexerStack1PwrCellTimeout)
        )
      )  */                                                              
//    );

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
