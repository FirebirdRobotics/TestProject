/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Drivetrain;

public class Autonomous extends SequentialCommandGroup {

  public Autonomous(Drivetrain drivetrain, AHRS gyro) {
    addCommands(
      new DriveDistanceEncoders(drivetrain, 24), // distance in inches
      new TurnToAngle(drivetrain, gyro, AutonomousConstants.kTurnSpeed, -90),
      new DriveDistanceEncoders(drivetrain, 24),
      new TurnToAngle(drivetrain, gyro, AutonomousConstants.kTurnSpeed, 0),
      new DriveDistanceEncoders(drivetrain, 24)
      );
  }
}
