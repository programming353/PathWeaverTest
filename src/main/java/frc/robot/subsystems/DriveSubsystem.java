// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax leftFrontMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(0, MotorType.kBrushless);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  private final RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private final RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  private final RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();

  private final AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

  private final DifferentialDriveOdometry m_odometry;

  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    leftFrontEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    rightFrontEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    leftFrontEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    rightFrontEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        gyro.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
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
    return new DifferentialDriveWheelSpeeds(leftFrontEncoder.getVelocity(), rightFrontEncoder.getVelocity());
  }

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

    /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }
  
    /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
      leftFrontEncoder.setPosition(0);
      rightFrontEncoder.setPosition(0);
    }
  
    public double getAverageEncoderDistance() {
      return (leftFrontEncoder.getPosition() + rightFrontEncoder.getPosition()) / 2.0;
    }

      /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return leftFrontEncoder;
  }

    /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return rightFrontEncoder;
  }

    /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }
}
