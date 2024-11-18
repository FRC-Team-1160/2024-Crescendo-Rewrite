// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TransportConstants;

public abstract class DriveTrain extends SubsystemBase {

  public SwerveModule[] m_modules;

  public SwerveDriveKinematics m_kinematics;

  public DriveTrain(){
    double offset = 23.75 * 0.0254; //MOVE TO CONSTANTS

    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(offset, offset), //front left
      new Translation2d(offset, -offset), //front right
      new Translation2d(-offset, offset), //back left
      new Translation2d(-offset, -offset) //back right
    );

    m_modules = new SwerveModule[4];
    m_modules[0] = initializeModule(1, 1, 1); // FIX PORTS
    m_modules[1] = initializeModule(2, 2, 2);
    m_modules[2] = initializeModule(3, 3, 3);
    m_modules[3] = initializeModule(4, 4, 4);
  }

  abstract SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port);

  public void setSwerveDrive(double x_metersPerSecond, double y_metersPerSecond, double a_radiansPerSecond){
    //converts speeds from field's frame of reference to robot's frame of reference
    ChassisSpeeds chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      x_metersPerSecond, 
      y_metersPerSecond, 
      a_radiansPerSecond, 
      getGyroAngle());

    setSwerveDrive(chassis_speeds);
  }

  public void setSwerveDrive(ChassisSpeeds chassis_speeds){
    //fix weird change over time shenanigans
    chassis_speeds = discretize_chassis_speeds(chassis_speeds);

    SwerveModuleState[] module_states = (m_kinematics.toSwerveModuleStates(chassis_speeds));

    //change target wheel directions if the wheel has to rotate more than 90*
    for (int i = 0; i < module_states.length; i++){
      module_states[i] = SwerveModuleState.optimize(module_states[i], m_modules[i].getAngle());
    }

    //normalize wheel speeds of any are greater than max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(module_states, SwerveConstants.MAX_SPEED); 

    setModules(module_states);
  }

  public void setModules(SwerveModuleState[] module_states){
    for (int i = 0; i < m_modules.length; i++){
      m_modules[i].setState(module_states[i]);
    }
  }

  //Thanks to Team 4738 for modified discretize code
  public ChassisSpeeds discretize_chassis_speeds(ChassisSpeeds speeds) {
    double dt = RobotConstants.LOOP_TIME_SECONDS; 
    //makes a Pose2d for the target delta over one time loop
    var desiredDeltaPose = new Pose2d(
      speeds.vxMetersPerSecond * dt, 
      speeds.vyMetersPerSecond * dt, 
      new Rotation2d(speeds.omegaRadiansPerSecond * dt * 1) //tunable
    );
    //makes a Twist2d object that maps new pose to delta pose
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  public SwerveModulePosition[] getModulePositions(){
    var positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++){
      positions[i] = m_modules[i].getModulePosition();
    }
    return positions;
  }


  public abstract Rotation2d getGyroAngle();

  @Override
  public void periodic() {
    for (SwerveModule module : m_modules){
      module.update();
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
