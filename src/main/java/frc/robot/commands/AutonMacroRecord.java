package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MacroRecorder;
import java.io.IOException;

public class AutonMacroRecord extends CommandBase {

    private DriveTrain m_drivetrain;
    private Intake m_intake;
    private Indexer m_indexer;
    private String m_filename;
    private MacroRecorder m_recorder;

    public AutonMacroRecord( String filename, DriveTrain drivetrain, Intake intake, Indexer indexer)
    {
        super();
        m_filename = filename;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_indexer = indexer;
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    try 
    {
        m_recorder = new MacroRecorder( m_filename, m_drivetrain, m_intake, m_indexer);
    }
    catch( IOException e )
    {
        e.printStackTrace();
    }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try 
    {
        m_recorder.record();
    }
    catch( IOException e )
    {
        e.printStackTrace();
    }
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
    try 
    {
        m_recorder.end();
    }
    catch( IOException e )
    {
        e.printStackTrace();
    }
  }

}
