/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakePositionPowerCellCommand extends CommandBase {
  private Intake m_intake;
  //private int intakeDelay;
  private boolean intakeHysteresis = true;
  /**
   * Creates a new IntakePositionPowerCellCommand.
   */
  public IntakePositionPowerCellCommand(Intake intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setSpeed(Constants.intakeMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //nothing here intentionally: will then run IntakeRunForABit
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_intake.getDistanceToPowerCell() < Constants.howCloseIsThePowerCell /*&& intakeHysteresis == true*/){
      //intakeHysteresis = false;
      return true;
    }
  // else if (m_intake.getDistanceToPowerCell() > 18 /*&& intakeHysteresis == false*/) {
    //intakeHysteresis = true;
     // return false;
   // }
    else {
      return false;
    }
/*
      intakeDelay = intakeDelay + 1;
      if( intakeDelay > 50 )
      {
        intakeDelay = 0;
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      intakeDelay = 0;
      return false;
    } 
    */
  }
}
