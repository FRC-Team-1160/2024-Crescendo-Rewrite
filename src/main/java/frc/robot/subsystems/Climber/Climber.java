// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;

public abstract class Climber extends SubsystemBase {

  /**Instance of the Climber.Commands class. */
  public Commands commands;

  public Climber() {
    commands = new Commands();
  }

  enum ClimberDirection {
    kUP,
    kDOWN,
    kOFF
  }

  public void setLeftClimber(int direction){ //-1, 0, 1
    if (!getLeftSwitch()) setLeftMotor(ClimberConstants.MOTOR_SPEED_VOLTS * direction);
  }

  public void setRightClimber(int direction){ //-1, 0, 1
    if (!getRightSwitch()) setRightMotor(-ClimberConstants.MOTOR_SPEED_VOLTS * direction);  
  }

  abstract void setLeftMotor(double speed);

  abstract void setRightMotor(double speed);

  abstract boolean getLeftSwitch();

  abstract boolean getRightSwitch();

  public class Commands {

    public Command setLeftDir(int direction){
      return new StartEndCommand(
        () -> setLeftClimber(direction),
        () -> setLeftClimber(0));
    }

    public Command setRightDir(int direction){
      return new StartEndCommand(
        () -> setRightClimber(direction),
        () -> setRightClimber(0));
    }

  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
