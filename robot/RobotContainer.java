/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AngleTesting;
import frc.robot.commands.Autonomous;
import frc.robot.commands.DriveDistanceEncoders;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OrchestraSystem;
import frc.robot.subsystems.VisionSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final Drivetrain m_drivetrain = new Drivetrain();
  private static final VisionSystem m_visionSystem = new VisionSystem();
  // private static final OrchestraSystem m_orchestra = new
  // OrchestraSystem(m_drivetrain);
  private final XboxController m_driverController = new XboxController(OIConstants.driverXboxPort);
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final Autonomous m_autoCommand = new Autonomous(m_drivetrain, m_gyro);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public double m_speedy = 0.2;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_gyro.enableBoardlevelYawReset(true);
    m_gyro.reset();

    // DRIVETRAIN
    m_drivetrain.setDefaultCommand(new RunCommand(() -> {
      m_drivetrain.autoDrive(-m_driverController.getY(Hand.kLeft) * m_speedy,
          m_driverController.getX(Hand.kLeft) * m_speedy);
      SmartDashboard.putNumber("Current Driving Speed", m_drivetrain.getVelocity() / MotorConstants.kFalconRPM);
    }, m_drivetrain));
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public VisionSystem getVisionSystem() {
    return m_visionSystem;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVE
    new JoystickButton(m_driverController, Button.kX.value).whenPressed(new DriveDistanceEncoders(m_drivetrain, 24),
        true);
    new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(new TurnToAngle(m_drivetrain, m_gyro, 0.2, 180));
    new JoystickButton(m_driverController, Button.kBumperLeft.value).whenPressed(() -> m_speedy -= 0.05);
    new JoystickButton(m_driverController, Button.kBumperRight.value).whenPressed(() -> m_speedy += 0.05);

    // VISION SYSTEM
    new JoystickButton(m_driverController, Button.kB.value)
        .whileHeld(new InstantCommand(() -> m_visionSystem.turnToTarget(m_drivetrain), m_drivetrain, m_visionSystem))
        .whenReleased(
            new InstantCommand(() -> m_visionSystem.visionRoutineReleased(m_drivetrain), m_drivetrain, m_visionSystem));

    // new JoystickButton(m_driverController, Button.kB.value).whileHeld(new RunCommand(new ShooterCommand (m_drivetrain, m_visionSystem.rawDistanceToTarget())), m_visionSystem);

    // AUTONOMOUS
    m_chooser.setDefaultOption("Auto 1", m_autoCommand);
    // m_chooser.addOption(name, object);

    // ORCHESTRA
    // new JoystickButton(m_driverController, Button.kStart.value)
    // .whenPressed(() -> m_orchestra.togglePauseMusic());
    // new JoystickButton(m_driverController, Button.kBack.value)
    // .whenPressed(() -> m_orchestra.toggleStopMusic());
    // new JoystickButton(m_driverController, Button.kBumperRight.value)
    // .whenPressed(() -> m_orchestra.nextSong());
    // new JoystickButton(m_driverController, Button.kBumperLeft.value)
    // .whenPressed(() -> m_orchestra.previousSong());

    SmartDashboard.putData("Autonomous", m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
