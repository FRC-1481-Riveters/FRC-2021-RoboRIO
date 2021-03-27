package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import frc.robot.subsystems.DriveTrain;

/*Code outline to implement playing back a macro recorded in BTMacroRecord
*Be sure to read out of the same file created in BTMacroRecord
*BEWARE OF: setting your motors in a different order than in BTMacroRecord and changing motor values before
*time is up. Both issues are dealt with and explained below. Also only read/write from/to the motors 
*you have fully coded for, otherwise your code will cut out for no reason. 
*In main, the try/catch structure catches any IOExceptions or FileNotFoundExceptions. Necessary to play back
*the recorded routine during autonomous
*Dennis Melamed and Melanie (sorta, she slept)
*March 22nd, 2015
*/


public class MacroPlayback {
	Scanner scanner;
	long startTime;
    DriveTrain m_drivetrain;
	boolean onTime = true;
    double nextDouble;
    double current_time;
    double current_left;
    double current_right;
    double next_time;
    double next_left;
    double next_right;
    boolean m_finished;

	public MacroPlayback( String filename, DriveTrain drivetrain ) throws FileNotFoundException
	{
		//create a scanner to read the file created during BTMacroRecord
		//scanner is able to read out the doubles recorded into recordedAuto.csv (as of 2015)
        
        scanner = new Scanner(new File( filename ));
        
        m_drivetrain = drivetrain;
        next_time  = -99999;

		//let scanner know that the numbers are separated by a comma or a newline, as it is a .csv file
		scanner.useDelimiter(",|\\n");
		
		//lets set start time to the current time you begin autonomous
        startTime = System.currentTimeMillis();	
        
        m_finished =  false;
	}
	
	public void play()
	{
        double elapsed_time;

        elapsed_time = System.currentTimeMillis()-startTime;

		//if recordedAuto.csv has a double to read next, then read it
		if ((scanner != null) && (scanner.hasNextDouble()))
		{
            while( next_time < elapsed_time)
            {
                current_time  = next_time;
                current_left = next_left;
                current_right = next_right;

                next_time = scanner.nextDouble();
                next_left = scanner.nextDouble();
                next_right = scanner.nextDouble();
            }

            if( current_time <= elapsed_time)
            {
                m_drivetrain.driveTank(current_left, current_right);
            }
			
		}
		//end play, there are no more values to find
		else
		{
			this.end();
			if (scanner != null) 
			{
				scanner.close();
				scanner = null;
			}
		}
		
	}
	
	//stop motors and end playing the recorded file
	public void end()
	{
        m_drivetrain.driveTank( 0, 0 );

        if (scanner != null)
		{
			scanner.close();
		}

        m_finished = true;
    }
    
    public boolean isFinished()
    {
        return m_finished;
    }
	
}