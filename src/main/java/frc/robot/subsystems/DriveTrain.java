/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

//import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.geometry.*;

public class DriveTrain extends SubsystemBase implements DoubleSupplier {
  /**
   * The DriveTrain subsystem incorporates the actuators attached to the robots
   * chassis.
   */
  // TalonSRX _leftMaster = new TalonSRX(15);
  // TalonSRX _rightMaster = new TalonSRX(1);
  CANSparkMax m_leftLead = new CANSparkMax(Constants.frontLeftMotor, MotorType.kBrushless);
  CANSparkMax m_leftFollower = new CANSparkMax(Constants.rearLeftMotor, MotorType.kBrushless);
  CANSparkMax m_rightLead = new CANSparkMax(Constants.frontRightMotor, MotorType.kBrushless);
  CANSparkMax m_rightFollower = new CANSparkMax(Constants.rearRightMotor, MotorType.kBrushless);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLead, m_rightLead);

  protected CANEncoder m_leftDriveEncoder;
  protected CANEncoder m_rightDriveEncoder;

  //NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  //NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");


  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  public final DifferentialDriveOdometry m_odometry;

  private double previousLeftPosition;
  private double previousRightPosition;
  private double leftVelocity;
  private double rightVelocity;
  private long previousFPGATime;

  /**
   * Create a new drive train subsystem.
   */
  public DriveTrain() {
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    m_leftFollower.restoreFactoryDefaults();
    m_leftLead.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();

    m_leftLead.setIdleMode(IdleMode.kBrake);
    m_rightLead.setIdleMode(IdleMode.kBrake);
    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_rightFollower.setIdleMode(IdleMode.kCoast);
   
    m_leftLead.setClosedLoopRampRate(Constants.closedLoopRampRate);
    m_rightLead.setClosedLoopRampRate(Constants.closedLoopRampRate);

    // Possible workaround for stutter per:
    //   https://www.chiefdelphi.com/t/spark-max-follower-with-lower-can-id-than-leader-causes-4-stutters-sec-until-power-cycled/378716/12
    m_leftLead.follow(ExternalFollower.kFollowerDisabled, 0);

    m_leftFollower.follow(m_leftLead);
    m_rightFollower.follow(m_rightLead);

    m_leftDriveEncoder = m_leftLead.getEncoder();
    m_rightDriveEncoder = m_rightLead.getEncoder();
    m_leftDriveEncoder.setPositionConversionFactor(1.0 / (Constants.drivetrainGearing / Constants.drivetrainWheelRevPerMeter));
    m_leftDriveEncoder.setVelocityConversionFactor((Constants.drivetrainWheelRevPerMeter / Constants.drivetrainGearing) / 60.0);
    m_rightDriveEncoder.setPositionConversionFactor(1.0 / (Constants.drivetrainGearing / Constants.drivetrainWheelRevPerMeter));
    m_rightDriveEncoder.setVelocityConversionFactor((Constants.drivetrainWheelRevPerMeter / Constants.drivetrainGearing) / 60.0);
    
    SmartDashboard.putNumber("leftVolts", 0.0);
    SmartDashboard.putNumber("rightVolts", 0.0);
  }

public void setOpenLoop(boolean auton)
{
  double rampRate;
  if (auton)
    rampRate = Constants.driveMotorRampRateAuto;
  else
    rampRate = Constants.driveMotorRampRateTele;

  m_leftLead.setOpenLoopRampRate( rampRate );
  m_leftFollower.setOpenLoopRampRate( rampRate );
  m_rightLead.setOpenLoopRampRate( rampRate );
  m_rightFollower.setOpenLoopRampRate( rampRate ); 
}

  /**
   * Arcade drive method
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                  positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Clockwise is positive.
   */
  public void drive(double xSpeed, double zRotation) {
    m_drive.arcadeDrive(xSpeed, zRotation);
    //System.out.println ("drive speed = " + xSpeed + ", rotation  = " + zRotation);
  }

  public void driveTank(double left, double right) {
    m_leftLead.set(left);
    m_rightLead.set(right);
    m_drive.feed();
    //System.out.println ("driveTank left  = " + -left + ", right  = " + right);
  }
  /**
   * Tank drive at a PID-driven velocity method.
   *
   */
  public void driveAtSpeed(double leftSpeedInMetersPerSecond, double rightSpeedInMetersPerSecond) {

  }
public void coastDrive(){
  m_leftLead.setIdleMode(IdleMode.kCoast);
  m_rightLead.setIdleMode(IdleMode.kCoast);
}
public void brakeDrive(){
  m_leftLead.setIdleMode(IdleMode.kBrake);
  m_rightLead.setIdleMode(IdleMode.kBrake);
}

  protected double meterPerSecondToRPM(double metersPerSecond) {
    double RPM;

    try {
      double metersPerMinute = metersPerSecond * 60.0;
      double wheelRevolutionsPerMeter = 1.0 / (Constants.driveTireDiameterInMeters * Math.PI);

      RPM = metersPerMinute * wheelRevolutionsPerMeter;

    } catch (Exception ex) {
      RPM = 0.0;
    }

    return RPM;
  }

  @Override
  public void periodic() {
    double leftPosition;
    double rightPosition;
    double scaleToSeconds;
    long fpgaTime;

    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    leftPosition = -m_leftDriveEncoder.getPosition();
    rightPosition = m_rightDriveEncoder.getPosition();

    // Improve velocity accuracy by using the FPGA time instead of a simple 20 ms or 50 Hz
    //fpgaTime = RobotController.getFPGATime();
    // FPGA time is in microseconds
    // divide one second by the time delta to scale the encoder values up to meters/sec
    //scaleToSeconds = 1000000.0 / (fpgaTime - previousFPGATime);
    //leftVelocity = (leftPosition - previousLeftPosition) * scaleToSeconds;
    //rightVelocity = (rightPosition - previousRightPosition) * scaleToSeconds;
    leftVelocity = -m_leftDriveEncoder.getVelocity();
    rightVelocity = m_rightDriveEncoder.getVelocity();

    previousLeftPosition = leftPosition;
    previousRightPosition = rightPosition;
    //previousFPGATime = fpgaTime;

    m_odometry.update( m_gyro.getRotation2d(), leftPosition, rightPosition );

    //SmartDashboard.putNumber("Left Encoder", -m_leftDriveEncoder.getPosition());
    //SmartDashboard.putNumber("Right Encoder", m_rightDriveEncoder.getPosition());
    //SmartDashboard.putNumber("gyro degrees",  getHeading());
    //SmartDashboard.putNumber("odo degrees",   m_odometry.getPoseMeters().getRotation().getDegrees());
    //SmartDashboard.putNumber("left velocity", leftVelocity );   //-m_leftDriveEncoder.getVelocity());
    //SmartDashboard.putNumber("right velocity", rightVelocity ); //m_rightDriveEncoder.getVelocity());

    //var translation = m_odometry.getPoseMeters().getTranslation();
    SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( leftVelocity, rightVelocity );
//    return new DifferentialDriveWheelSpeeds(-m_leftDriveEncoder.getVelocity(), m_rightDriveEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.reset();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //SmartDashboard.putNumber("leftVolts", -leftVolts);
    //SmartDashboard.putNumber("rightVolts", rightVolts);
    m_leftLead.setVoltage(-leftVolts);
    m_rightLead.setVoltage(rightVolts);
    //System.out.println ("tankDriveVolts left  = " + leftVolts + ", right  = " + rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {
    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
  }

  public double getVoltsLeft()
  {
     return m_leftLead.get();
  }

  public double getVoltsRight()
  {
     return m_rightLead.get();
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /*
   * Compute the distance the robot has travelled since its last reset
   *
   * @return distanceTravelled The cumulative distance the robot has travelled.
   * Reference Constants.driveTrainInchesPerEncoderCounts for units
   * 
   * Note that the motors and encoders on the right side of the robot are, by
   * convention, considered to be running backwards. Compensate for this by
   * subtracing this negative number. This way, you're subtracting a negative
   * value, which is adding a positive value!
   */
  @Override
  public double getAsDouble() {
    double averageNEORevolutionsTravelled = (-m_leftDriveEncoder.getPosition() + m_rightDriveEncoder.getPosition())
        / 2.0;

    return (averageNEORevolutionsTravelled * Constants.driveTrainInchesPerEncoderCounts);
  }
}
