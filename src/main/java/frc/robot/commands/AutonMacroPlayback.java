package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MacroPlayback;
import java.io.IOException;

public class AutonMacroPlayback extends CommandBase {

    private DriveTrain m_drivetrain;
    private Intake m_intake;
    private Indexer  m_indexer;
    private String m_filename;
    private MacroPlayback m_playback;

    public AutonMacroPlayback( String filename, DriveTrain drivetrain, Intake intake, Indexer indexer)
    {
        super();
        m_filename = filename;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_indexer = indexer;
        addRequirements(drivetrain);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_drivetrain.setOpenLoop(false);
    try 
    {
        m_playback = new MacroPlayback( m_filename, m_drivetrain, m_intake, m_indexer );
    }
    catch( IOException e )
    {
        e.printStackTrace();
    }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_playback.play();
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_playback.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end( interrupted );
    m_drivetrain.driveTank(0, 0);
    m_intake.setIntake(0);
    m_indexer.setIndexer(0,0);
  }

}
