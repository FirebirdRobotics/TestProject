/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {

  private Drivetrain m_drive;
  private double m_rotationSpeed;
  private double m_targetAngle;
  private AHRS m_gyro;

  private double m_gyroNearestAngle;

  private boolean isFinished;

  private double error;
  private double integral;
  private double derivative;
  private double previousError;

  private final double kP = 0.088;
  private final double kI = 0.010;
  private final double kD = 0.026;
  
  // Angle needs to be in radians, as that is the superior unit mathematically speaking. (lol)
  // RotateSpeed, on the other hand, is in ratio form (as used on all SpeedControllers)
  public TurnToAngle(Drivetrain dt, AHRS gyro, double rotationSpeed, double targetAngle) {
    m_drive = dt;
    m_rotationSpeed = rotationSpeed;
    m_gyro = gyro;
    m_targetAngle = targetAngle;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    integral = 0;
    previousError = 0;

    System.out.println("starting command...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("target angle: " + m_targetAngle);
    // System.out.println("current angle: " + m_gyro.getAngle());
    System.out.println("running command...");

    error = m_targetAngle - m_gyro.getAngle(); // error = target - actual
    integral += error * 0.02; // integral increased by error * time (0.02 seconds per loop)
    derivative = (error - previousError) / .02; // derivative = change in error / time (0.02 seconds per loop)
    previousError = error; 

    m_rotationSpeed = (error * kP) + (integral * kI) + (derivative * kD);
    m_rotationSpeed /= 35;

    m_gyroNearestAngle = Math.round(m_gyro.getAngle());

    if (m_gyroNearestAngle < m_targetAngle) {
      m_rotationSpeed += .12;
      m_drive.autoDrive(0, m_rotationSpeed);
      isFinished = false;
    } else if (m_gyroNearestAngle > m_targetAngle) {
      m_rotationSpeed -= .12;
      m_drive.autoDrive(0, m_rotationSpeed);
      isFinished = false;
    } else {
      m_drive.autoDrive(0, 0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ending command.");
    error = 0;
    integral = 0;
    derivative = 0;
    previousError = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
