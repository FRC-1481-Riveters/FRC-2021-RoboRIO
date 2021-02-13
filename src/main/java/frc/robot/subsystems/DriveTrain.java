/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  protected CANPIDController m_leftPIDController;
  protected CANPIDController m_rightPIDController;

  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SerialPort.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Create a new drive train subsystem.
   */
  public DriveTrain() {
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    m_leftFollower.restoreFactoryDefaults();
    m_leftLead.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();

    m_leftLead.setIdleMode(IdleMode.kCoast);
    m_leftLead.setOpenLoopRampRate(Constants.driveMotorRampRate); // numbers = seconds until full speed

    m_rightLead.setIdleMode(IdleMode.kCoast);
    m_rightLead.setOpenLoopRampRate(Constants.driveMotorRampRate); // numbers = seconds until full speed

    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_leftFollower.setOpenLoopRampRate(Constants.driveMotorRampRate); // numbers = seconds until full speed

    m_rightFollower.setIdleMode(IdleMode.kCoast);
    m_rightFollower.setOpenLoopRampRate(Constants.driveMotorRampRate); // numbers = seconds until full speed

    m_leftLead.setClosedLoopRampRate(Constants.closedLoopRampRate);
    m_rightLead.setClosedLoopRampRate(Constants.closedLoopRampRate);

    m_leftFollower.follow(m_leftLead);
    m_rightFollower.follow(m_rightLead);

    m_leftDriveEncoder = m_leftLead.getEncoder();
    m_rightDriveEncoder = m_rightLead.getEncoder();
    m_leftDriveEncoder.setPositionConversionFactor(1.0 / (Constants.drivetrainGearing / Constants.drivetrainWheelRevPerMeter));
    m_rightDriveEncoder.setPositionConversionFactor(1.0 / (Constants.drivetrainGearing / Constants.drivetrainWheelRevPerMeter));
    
    m_leftPIDController = m_leftLead.getPIDController();
    m_rightPIDController = m_rightLead.getPIDController();

    m_leftPIDController.setP(Constants.kDriveGains.kP);
    m_leftPIDController.setI(Constants.kDriveGains.kI);
    m_leftPIDController.setD(Constants.kDriveGains.kD);
    m_leftPIDController.setFF(Constants.kDriveGains.kF);

    m_rightPIDController.setP(Constants.kDriveGains.kP);
    m_rightPIDController.setI(Constants.kDriveGains.kI);
    m_rightPIDController.setD(Constants.kDriveGains.kD);
    m_rightPIDController.setFF(Constants.kDriveGains.kF);
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
  }

  /**
   * Tank drive at a PID-driven velocity method.
   *
   */
  public void driveAtSpeed(double leftSpeedInMetersPerSecond, double rightSpeedInMetersPerSecond) {

    /*
     * The Spark Maxs by default deal with speeds in RPM. So, convert the meters per
     * second that this routine accepts for each of the motor speeds into RPM before
     * telling each of the motors to spin at that speed.
     */

    double leftMotorRPM = meterPerSecondToRPM(leftSpeedInMetersPerSecond);
    m_leftPIDController.setReference(-leftMotorRPM, ControlType.kVelocity);

    /*
     * By convention, the Right motor is inverted. So, invert this motor while under
     * direction control of this PID
     */
    double rightMotorRPM = meterPerSecondToRPM(-1.0 * rightSpeedInMetersPerSecond);
    m_rightPIDController.setReference(-rightMotorRPM, ControlType.kVelocity);
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
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftDriveEncoder.getPosition(),
                      -m_rightDriveEncoder.getPosition());
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    SmartDashboard.putNumber("Left Drive", m_leftDriveEncoder.getPosition());
    SmartDashboard.putNumber("Right Drive", -m_rightDriveEncoder.getPosition());
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
    return new DifferentialDriveWheelSpeeds(m_leftDriveEncoder.getVelocity(), -m_rightDriveEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLead.setVoltage(leftVolts);
    m_rightLead.setVoltage(-rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {
    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftDriveEncoder.getPosition() + m_rightDriveEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftDriveEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightDriveEncoder;
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
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
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
    double averageNEORevolutionsTravelled = (m_leftDriveEncoder.getPosition() - m_rightDriveEncoder.getPosition())
        / 2.0;

    return (averageNEORevolutionsTravelled * Constants.driveTrainInchesPerEncoderCounts);
  }
}
