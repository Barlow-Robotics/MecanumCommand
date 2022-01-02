// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS; //where 2 get this library ?
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
import frc.robot.PhysicsSim; //where 2 get this library ?
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;

public class DriveSubsystem extends SubsystemBase {
  
    WPI_TalonSRX m_frontLeft;
    WPI_TalonSRX m_rearLeft;
    WPI_TalonSRX m_frontRight;
    WPI_TalonSRX m_rearRight;

    private final MecanumDrive m_drive;

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    MecanumDriveOdometry m_odometry =
        new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        m_frontLeft = new WPI_TalonSRX(DriveConstants.ID_frontLeftMotor) ;
        m_frontRight = new WPI_TalonSRX(DriveConstants.ID_frontRightMotor) ;
        m_rearLeft = new WPI_TalonSRX(DriveConstants.ID_rearLeftMotor) ;
        m_rearRight = new WPI_TalonSRX(DriveConstants.ID_rearRightMotor) ; 

        m_drive =  new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

        m_frontLeft.configFactoryDefault() ;
        m_rearLeft.configFactoryDefault() ;
        m_frontRight.configFactoryDefault() ;
        m_rearRight.configFactoryDefault() ;
    
        m_rearLeft.setInverted(false);
        m_frontLeft.setInverted(false);
        m_rearLeft.follow(m_frontLeft);
        //initializePIDConfig(leftFrontSide);
    
        m_rearRight.setInverted(false);
        m_frontRight.setInverted(false);
        m_rearRight.follow(m_frontRight);
        //initializePIDConfig(rightFrontSide);  what 2 do abt  this ??

        // Sets the distance per pulse for the encoders

        // m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update( //needs to use .getwheelspeeds, but we only create it later on,,,
        m_gyro.getRotation2d(), 
        // m_rearLeft.getSelectedSensorPosition(), 
        // m_rearRight.getSelectedSensorPosition()
        // );
        getCurrentWheelSpeeds()
        );
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
  public MecanumDriveWheelSpeeds getWheelSpeeds() {  //why does this require 4 inputs ?
    return new MecanumDriveWheelSpeeds(m_frontLeft.getSelectedSensorVelocity(), m_frontRight.getSelectedSensorVelocity(), m_rearLeft.getSelectedSensorVelocity(), m_rearRight.getSelectedSensorVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }
  }

  /** Sets the front left drive SpeedController to a voltage. */
  public void setDriveSpeedControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_rearRight.setSelectedSensorPosition(0, 0, 0);
    m_rearLeft.setSelectedSensorPosition(0, 0, 0);
  }

    /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_rearLeft.getSelectedSensorPosition() + m_rearRight.getSelectedSensorPosition()) / 2.0;
  }

  // /**
  //  * Gets the front left drive encoder.
  //  *
  //  * @return the front left drive encoder
  //  */
  // public Encoder getFrontLeftEncoder() {
  //   return m_frontLeftEncoder;
  // }

  // /**
  //  * Gets the rear left drive encoder.
  //  *
  //  * @return the rear left drive encoder
  //  */
  // public Encoder getRearLeftEncoder() {
  //   return m_rearLeftEncoder;
  // }

  // /**
  //  * Gets the front right drive encoder.
  //  *
  //  * @return the front right drive encoder
  //  */
  // public Encoder getFrontRightEncoder() {
  //   return m_frontRightEncoder;
  // }

  // /**
  //  * Gets the rear right drive encoder.
  //  *
  //  * @return the rear right encoder
  //  */
  // public Encoder getRearRightEncoder() {
  //   return m_rearRightEncoder;
  // }

  //why use that ??  ↓
  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {

    // Multiplying ny 10.0 because Talon reports velocity as counts per 100 mSec.
    return new MecanumDriveWheelSpeeds(
      m_frontLeft.getSelectedSensorVelocity() * 10.0 ,
      m_rearLeft.getSelectedSensorVelocity() * 10.0 ,
      m_frontRight.getSelectedSensorVelocity() * 10.0 ,
      m_rearRight.getSelectedSensorVelocity() * 10.0 ) ;
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

  
  private void initializePIDConfig(WPI_TalonSRX talon){
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.DriveConstants.mainFeedbackLoop, Constants.DriveConstants.timeoutTime); //Encoder as feedback device, main PID loop, 30 ms timeout time
    talon.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant);
    talon.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant);
    talon.configNominalOutputForward(0);
    talon.configNominalOutputReverse(0);
    talon.configPeakOutputForward(1.0);
    talon.configPeakOutputReverse(-1.0);
    talon.configMotionCruiseVelocity((int)(Constants.DriveConstants.unitsPerRotation * Constants.DriveConstants.desiredRPMsForDrive));
    talon.config_kF(Constants.DriveConstants.PID_id, Constants.DriveConstants.DrivetrainKf);
    talon.config_kP(Constants.DriveConstants.PID_id, Constants.DriveConstants.DrivetrainkP);
    talon.config_kI(Constants.DriveConstants.PID_id, 0);
    talon.config_kD(Constants.DriveConstants.PID_id, 0);
  }

  private void simulationInit() {
      PhysicsSim.getInstance().addTalonSRX(m_frontRight, 0.75, 4000, true);
      PhysicsSim.getInstance().addTalonSRX(m_frontLeft, 0.75, 4000, true);
      PhysicsSim.getInstance().addTalonSRX(m_rearRight, 0.75, 4000);
      PhysicsSim.getInstance().addTalonSRX(m_rearLeft, 0.75, 4000);
  }

  public double getDistance(){
    double leftPos = m_frontLeft.getSelectedSensorPosition();
    double rightPos = m_frontRight.getSelectedSensorPosition();
    double averagePos = (leftPos + rightPos)/2;
    return averagePos;
  }

  public void resetDistance(){
    m_frontLeft.setSelectedSensorPosition(0.0);
    m_frontRight.setSelectedSensorPosition(0.0);
  }

}
