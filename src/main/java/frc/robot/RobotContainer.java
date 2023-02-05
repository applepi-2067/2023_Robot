// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.waist.*;
import frc.robot.subsystems.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.shoulder.*;
import io.github.oblarg.oblog.Logger;
import frc.robot.commands.auto.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.arm.*;

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

  // private final ExampleSubsystem example = ExampleSubsystem.getInstance();
  private final Drivetrain m_robotDrive = new Drivetrain();
  private final Waist m_waist = Waist.getInstance();
  private final Shoulder m_shoulder = Shoulder.getInstance();
  private final Vision m_vision = new Vision();
  private final Arm m_arm = Arm.getInstance();

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
                  -m_driverController.getLeftY() / 2.0,
                  -m_driverController.getRightX() / 3.0
                ),
          m_robotDrive)
        );
  

    m_waist.setDefaultCommand(new DriveWaistWithJoystick(() -> m_operatorContoller.getLeftX()));
    m_shoulder.setDefaultCommand(new DriveShoulderWithJoystick(() -> m_operatorContoller.getRightY()));
    m_arm.setDefaultCommand(new DriveArmWithJoystick(() -> m_operatorContoller.getLeftY()));
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
    m_driverController.a().onTrue(new RotateToPosition( -90.0));
    m_driverController.b().onTrue(new RotateToPosition( 90));

    m_driverController.x().onTrue(new InstantCommand(() -> System.out.println(m_vision.getCameraAbsolutePose())));

    //Operator Controls
    //m_operatorContoller.a().onTrue(new SetWaistPosition(0));
    //m_operatorContoller.b().onTrue(new SetWaistPosition(10));
    m_operatorContoller.leftBumper().onTrue(new SetArmExtension(0));
    m_operatorContoller.rightBumper().onTrue(new SetArmExtension(0.5));
    
    m_operatorContoller.x().onTrue(new SetShoulderPosition(90));
    m_operatorContoller.y().onTrue(new SetShoulderPosition(270));
    m_operatorContoller.a().onTrue(new DriveShoulderWithJoystick(()->{return 0.0;}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int targetID = 2;
    Pose2d destinationTargetPose = new Pose2d(1, 0, new Rotation2d(Math.toRadians(180)));

    DriveToVisionTargetOffset m_autonomousCommand = new DriveToVisionTargetOffset(
      m_robotDrive, m_vision, targetID, destinationTargetPose
    );

    // RotationTest m_autonomousCommand = new RotationTest(m_robotDrive);
    // DriveSquareAuto m_autonomousCommand = new DriveSquareAuto(m_robotDrive);

    return m_autonomousCommand;
  }

  /**
   * Sets motors to coast or brake mode
   * @param coastEnabled Set coast enabled if true, otherwise set brake mode
   */
  public void setCoastEnabled(boolean coastEnabled) {
    if (coastEnabled) {
      m_robotDrive.setMotorsCoast();
    }
    else {
      m_robotDrive.setMotorsBrake();
    }
  }
}
