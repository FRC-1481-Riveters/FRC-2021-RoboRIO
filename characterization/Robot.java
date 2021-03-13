/**
* This is a very simple robot program that can be used to send telemetry to
* the data_logger script to characterize your drivetrain. If you wish to use
* your actual robot code, you only need to implement the simple logic in the
* autonomousPeriodic function and change the NetworkTables update rate
*/

/*
0.47798 meters per wheel revolution
10.11111 neo revolutions per wheel revolution
42 neo encoder pulses per neo revolution
424 neo encoder pulses per wheel revolution
887 neo encoder pulses per meter
0.001126 meters per pulse
Encoder position factor is 0.0472727
*/
package dc;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// WPI_Talon* imports are needed in case a user has a Pigeon on a Talon
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.AlternateEncoderType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList; 

public class Robot extends TimedRobot {

  static private int ENCODER_EDGES_PER_REV = 1 / 4;
  static private int PIDIDX = 0;
  static private double ENCODER_EPR = 42.0;
  static private double GEARING = ((42.0 * 56.0) / (18.0 * 11.0));
  static private double WHEEL_REV_PER_METER = 0.474788; 
  private double encoderConstant = (1 / GEARING);

  Joystick stick;
  DifferentialDrive drive;


  double leftEncoderPosition;
  double leftEncoderRate;
  double rightEncoderPosition;
  double rightEncoderRate;
  Supplier<Double> gyroAngleRadians;
  double leftEncoderPrevious;
  double rightEncoderPrevious;

  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  String data = "";
  
  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;

  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();
  public Robot() {
    super(.005);
    LiveWindow.disableAllTelemetry();
  }

  public enum Sides {
    LEFT,
    RIGHT,
    FOLLOWER
  }

  // methods to create and setup motors (reduce redundancy)
  public CANSparkMax setupCANSparkMax(int port, Sides side, boolean inverted) {
    // create new motor and set neutral modes (if needed)
    // setup Brushless spark
    CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
    motor.restoreFactoryDefaults(); 
    motor.setIdleMode(IdleMode.kBrake);  
    motor.setInverted(inverted);
    
    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {
    
      Encoder encoder;



 
    
    }
    

    return motor;

  }

  @Override
  public void robotInit() {
    if (!isReal()) SmartDashboard.putData(new SimEnabler());

    stick = new Joystick(0);
    
    // create left motor
    leftMotor = setupCANSparkMax(13, Sides.LEFT, true);

    leftMotor.getEncoder().setPositionConversionFactor( 1.0 / (GEARING / WHEEL_REV_PER_METER) );
    leftMotor.getEncoder().setVelocityConversionFactor( (WHEEL_REV_PER_METER / GEARING) / 60.0  );

    CANSparkMax leftFollowerID12 = setupCANSparkMax(12, Sides.FOLLOWER, true);
    leftFollowerID12.follow(leftMotor, false);
        

    rightMotor = setupCANSparkMax(2, Sides.RIGHT, false);

    rightMotor.getEncoder().setPositionConversionFactor( 1.0 / (GEARING / WHEEL_REV_PER_METER) );
    rightMotor.getEncoder().setVelocityConversionFactor( (WHEEL_REV_PER_METER / GEARING) / 60.0 );

    CANSparkMax rightFollowerID3 = setupCANSparkMax(3, Sides.FOLLOWER, false);
    rightFollowerID3.follow(rightMotor, false);
    drive = new DifferentialDrive(leftMotor, rightMotor);
    drive.setDeadband(0);


leftEncoderPosition = -1.0 * (leftMotor.getEncoder().getPosition());
leftEncoderPrevious = leftEncoderPosition;

rightEncoderPosition = rightMotor.getEncoder().getPosition();
rightEncoderPrevious = rightEncoderPosition;


    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of WPILib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    AHRS navx = new AHRS(SPI.Port.kMXP);
    gyroAngleRadians = () -> -1 * Math.toRadians(navx.getAngle());

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    drive.tankDrive(0, 0);
    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    System.out.println("Robot disabled");
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data = "";
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {

leftEncoderPosition = (leftMotor.getEncoder().getPosition());
leftEncoderRate = leftMotor.getEncoder().getVelocity();
leftEncoderPrevious = leftEncoderPosition;

rightEncoderPosition = rightMotor.getEncoder().getPosition();
rightEncoderRate = rightMotor.getEncoder().getVelocity();
rightEncoderPrevious = rightEncoderPosition;

    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition);
    SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate);
    SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition);
    SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate);
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  @Override
  public void teleopPeriodic() {
    drive.tankDrive(-stick.getY(), stick.getX());
  }

  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  /**
  * If you wish to just use your own robot program to use with the data logging
  * program, you only need to copy/paste the logic below into your code and
  * ensure it gets called periodically in autonomous mode
  * 
  * Additionally, you need to set NetworkTables update rate to 10ms using the
  * setUpdateRate call.
  */
  @Override
  public void autonomousPeriodic() {

    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = leftEncoderPosition;
    double leftRate = leftEncoderRate;

    double rightPosition = rightEncoderPosition;
    double rightRate = rightEncoderRate ;

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive.tankDrive(
      autospeed, -1.0 * autospeed,
      false
    );

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }
}
