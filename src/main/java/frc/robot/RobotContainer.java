// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainRealIO;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SubsystemManager m_subsystem_manager = new SubsystemManager();

  private Joystick m_mainStick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick m_codriverStick = new Joystick(Constants.IO.COPILOT_PORT);
  // private Joystick m_codriverSimpStick = new Joystick(Constants.IO.COPILOT_SIMP_PORT);
  private Joystick m_leftBoard = new Joystick(Constants.IO.LEFT_BOARD_PORT);
  private Joystick m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_leftBoard, Constants.IO.Board.Left.AIM)
        .whileTrue(m_subsystem_manager.commands.aimSpeaker());

    new JoystickButton(m_leftBoard, Constants.IO.Board.Left.REV)
        .whileTrue(m_subsystem_manager.commands.rev());

    new JoystickButton(m_leftBoard, Constants.IO.Board.Left.SHOOT)
        .onTrue(m_subsystem_manager.commands.shoot());

    new JoystickButton(m_rightBoard, Constants.IO.Board.Right.UP_DOWN_INTAKE)
        .whileTrue(m_subsystem_manager.m_intake.commands.setDefault());

    new JoystickButton(m_rightBoard, Constants.IO.Board.Right.INTAKE)
        .toggleOnTrue(m_subsystem_manager.commands.intake());

    new Trigger(() -> Math.abs(m_mainStick.getRawAxis(Constants.IO.Board.Left.LEFT_CLIMB)) > 0.5)
        .whileTrue(m_subsystem_manager.m_climber.commands.setLeftDir(
          (int) Math.signum(m_mainStick.getRawAxis(Constants.IO.Board.Left.LEFT_CLIMB))));
    
    new Trigger(() -> Math.abs(m_mainStick.getRawAxis(Constants.IO.Board.Right.RIGHT_CLIMB)) > 0.5)
        .whileTrue(m_subsystem_manager.m_climber.commands.setRightDir(
          (int) Math.signum(m_mainStick.getRawAxis(Constants.IO.Board.Right.RIGHT_CLIMB))));
  }

  public void updateSubsystemManager(){
    m_subsystem_manager.periodic(
      m_mainStick.getRawAxis(1), //REPLACE
      m_mainStick.getRawAxis(0), 
      m_codriverStick.getRawAxis(0));
  }

}
