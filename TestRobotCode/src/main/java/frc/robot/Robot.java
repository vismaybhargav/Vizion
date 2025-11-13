// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable visionTable = inst.getTable("vision");
  
  DoubleArraySubscriber tag67sub = visionTable.getDoubleArrayTopic("tag_67_pose_cam").subscribe(new double[0]);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else if(isSimulation()) {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.

    Logger.recordOutput("Cam Pose", new Pose3d());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double[] pose = tag67sub.get();
    if(pose.length >= 6) {
        double x = pose[0];
        double y = pose[1];
        double z = pose[2];
        double roll = pose[3];
        double pitch = pose[4];
        double yaw = pose[5];

        Pose3d tagPoseInCamera = new Pose3d(
          new Translation3d(x, y, z), 
          new Rotation3d(roll, pitch, yaw)
        );

        Logger.recordOutput("Tag Pose", tagPoseInCamera);
    }
  }

  public Rotation3d rotationFromRVec(double rx, double ry, double rz) {
    double mag = Math.sqrt(rx * rx + ry * ry + rz * rz);

    if (mag < 1e-9) {
      return new Rotation3d();
    }

    double ax = rx / mag;
    double ay = ry / mag;
    double az = rz / mag;

    double axWp = az;
    double ayWp = -ax;
    double azWp = -ay;

    double half = mag / 2.0;
    double sinHalf = Math.sin(half);

    Quaternion q = new Quaternion(
      axWp * sinHalf,
      ayWp * sinHalf,
      azWp * sinHalf,
      Math.cos(half)
    );

    return new Rotation3d(q);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
