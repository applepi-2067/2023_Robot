// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.waist.DriveWaistWithJoystick;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;

/**
 * This class is where the bulk of the robot should be declared. 
 * Since Command-based is a "declarative" paradigm, very little robot logic 
 * should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).
 * Instead, the robot structure (including subsystems, commands,
 * and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Instantiate subsystems, controllers, and commands.
  private final CommandXboxController m_driverController = new CommandXboxController(
    Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorContoller = new CommandXboxController(
    Constants.OperatorConstants.kOperatorControllerPort);

  private final Drivetrain m_robotDrive = new Drivetrain();
  private final Waist waist = Waist.getInstance();
  private final ExampleSubsystem example = ExampleSubsystem.getInstance();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);

    // Configure the trigger bindings.
    configureBindings();

    // Configure default commands.
    // Set the default drive command to split-stick arcade drive.
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command.
        // Forward/backward controlled by the left hand, turning controlled by the right.
        Commands.run(
          () -> m_robotDrive.arcadeDrive(
                  m_driverController.getLeftY() / 2.0,
                  m_driverController.getRightX() / 2.0
                ),
          m_robotDrive)
        );
  
    waist.setDefaultCommand(new DriveWaistWithJoystick(() -> m_operatorContoller.getLeftX()));
  }

  /**
   * Use this method to define your trigger-> command mappings.
   * Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
   * with an arbitrary predicate, or via the named factories in 
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses 
   * for {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    //Driver Controls

    //Operator Controls
    m_operatorContoller.a().onTrue(new SetWaistPosition(0));
    m_operatorContoller.b().onTrue(new SetWaistPosition(10));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous.
    return null;
  }
}