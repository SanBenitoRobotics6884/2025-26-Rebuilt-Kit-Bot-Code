// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Moters.DRIFT_LOWER_LIM;
import static frc.robot.Constants.Moters.DRIFT_UPPER_LIM;
import static frc.robot.Constants.Moters.INTAKE_SPEED;
import static frc.robot.Constants.Moters.IN_MOTER_ID;
import static frc.robot.Constants.Moters.MOTER_LEFT1_ID;
import static frc.robot.Constants.Moters.MOTER_LEFT2_ID;
import static frc.robot.Constants.Moters.OUTTAKE_SPEED;
import static frc.robot.Constants.Moters.OUT_MOTER_ID;
import static frc.robot.Constants.Moters.ROTATING_SPEED;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  XboxController m_Controller = new XboxController(0);

  SparkMax m_Moter_Left1 = new SparkMax(MOTER_LEFT1_ID, MotorType.kBrushless);
  SparkMax m_Moter_Left2 = new SparkMax(MOTER_LEFT2_ID, MotorType.kBrushless);
  SparkMax m_Moter_Right1 = new SparkMax(MOTER_LEFT1_ID, MotorType.kBrushless);
  SparkMax m_Moter_Right2 = new SparkMax(MOTER_LEFT2_ID, MotorType.kBrushless);

  TalonFX m_InMoter = new TalonFX(IN_MOTER_ID);
  TalonFX m_OutMoter = new TalonFX(OUT_MOTER_ID);

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Driving forward or backward
    if ((m_Controller.getLeftY() > DRIFT_UPPER_LIM) || (m_Controller.getLeftY() < DRIFT_LOWER_LIM)) {
      m_Moter_Left1.set(m_Controller.getLeftY());
      m_Moter_Left2.set(m_Controller.getLeftY());
      m_Moter_Right1.set(m_Controller.getLeftY());
      m_Moter_Right2.set(m_Controller.getLeftY());
    }

    // Rotating Right
    if (m_Controller.getRightX() > DRIFT_UPPER_LIM) {
      m_Moter_Left1.set(ROTATING_SPEED);
      m_Moter_Left2.set(ROTATING_SPEED);
      m_Moter_Right1.set(-ROTATING_SPEED);
      m_Moter_Right2.set(-ROTATING_SPEED);
    }
    // Rotating Left
    if (m_Controller.getRightX() < DRIFT_LOWER_LIM) {
      m_Moter_Left1.set(-ROTATING_SPEED);
      m_Moter_Left2.set(-ROTATING_SPEED);
      m_Moter_Right1.set(ROTATING_SPEED);
      m_Moter_Right2.set(ROTATING_SPEED);
    }

    // Reset when not driving
    if ((m_Controller.getLeftY() >= DRIFT_LOWER_LIM) || (m_Controller.getLeftY() <= DRIFT_UPPER_LIM)) {
      m_Moter_Left1.set(0);
      m_Moter_Left2.set(0);
      m_Moter_Right1.set(0);
      m_Moter_Right2.set(0);
    }
    // Reset when not rotating
    if ((m_Controller.getRightX() >= DRIFT_LOWER_LIM) || (m_Controller.getRightX() <= DRIFT_UPPER_LIM)) {
      m_Moter_Left1.set(0);
      m_Moter_Left2.set(0);
      m_Moter_Right1.set(0);
      m_Moter_Right2.set(0);
    }

    // Intake
    if (m_Controller.getRightBumperButtonPressed()) {
      m_InMoter.set(INTAKE_SPEED);
    }
    if (m_Controller.getRightBumperButtonReleased()) {
      m_InMoter.set(0);
    }

    // Outtake
    if (m_Controller.getRightTriggerAxis() > 0.1) {
      m_InMoter.set(-OUTTAKE_SPEED);
      m_OutMoter.set(OUTTAKE_SPEED);
    }
    if (m_Controller.getRightTriggerAxis() <= 0.1) {
      m_InMoter.set(0);
      m_OutMoter.set(0);
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
