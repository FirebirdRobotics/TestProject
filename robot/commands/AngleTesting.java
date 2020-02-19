/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Drivetrain;

public class AngleTesting extends CommandBase {
  
  private Drivetrain m_drive;
  private AHRS m_gyro;

  private double m_actualSpeed;
  private double m_targetSpeed;

  private double m_initialAngle;
  private double m_finalAngle;

  private boolean isFinished;

  public AngleTesting(Drivetrain drive, AHRS gyro, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gyro = gyro;
    m_drive = drive;
    m_targetSpeed = speed;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialAngle = m_gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get actual speed
    m_actualSpeed = m_drive.getVelocity() / MotorConstants.kFalconRPM;

    SmartDashboard.putNumber("actual speed",m_drive.getVelocity());

    if (m_actualSpeed < m_targetSpeed) {
      m_drive.autoDrive(0, m_targetSpeed);
      isFinished = false;
    } else if (m_actualSpeed >= m_targetSpeed){
      m_drive.autoDrive(0, 0);
      m_finalAngle = m_gyro.getAngle();
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Change in Angle", m_finalAngle - m_initialAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
