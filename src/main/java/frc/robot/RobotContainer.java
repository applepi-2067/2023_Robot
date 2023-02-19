// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.waist.*;
import frc.robot.subsystems.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.intake.ActivateIntakeRollers;
import frc.robot.commands.intake.IntakeConveyorIn;

import frc.robot.commands.shoulder.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
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
public class RobotContainer implements Loggable{
  // Instantiate subsystems, controllers, and commands.
  private final CommandXboxController m_driverController = new CommandXboxController(
    Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
    Constants.OperatorConstants.kOperatorControllerPort);

  private final Drivetrain m_robotDrive = Drivetrain.getInstance();
  private final Waist m_waist = Waist.getInstance();
  private final Shoulder m_shoulder = Shoulder.getInstance();
  private final Vision m_vision = Vision.getInstance();
  private final Arm m_arm = Arm.getInstance();
  private final ClawGrasp m_ClawGrasp = ClawGrasp.getInstance();
  private static DigitalInput m_practiceBotJumper = new DigitalInput(Constants.DiscreteInputs.PBOT_JUMPER_DI);
  private Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    m_compressor.enableDigital();
    // Configure the trigger bindings.
    configureBindings();

    // Configure default commands.
    // Set the default drive command to split-stick arcade drive.
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command.
        // Forward/backward controlled by the left hand, turning controlled by the right.
        Commands.run(
          () -> m_robotDrive.arcadeDrive(
                  -m_driverController.getLeftY() / 1.5,
                  -m_driverController.getRightX() / 2.0
                ),
          m_robotDrive)
        );
  

    m_waist.setDefaultCommand(new DriveWaistWithJoystick(() -> m_operatorController.getLeftX() / 4.0));
    m_shoulder.setDefaultCommand(new DriveShoulderWithJoystick(() -> m_operatorController.getRightY()));
    m_arm.setDefaultCommand(new DriveArmWithJoystick(() -> m_operatorController.getLeftY()));
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
    //m_operatorController.a().onTrue(new SetArmExtension(0.0));
    //m_operatorController.b().onTrue(new SetArmExtension(0.5));
    m_operatorController.a().onTrue(new ActivateIntakeRollers(true));
    
    

    // m_operatorContoller.a().onTrue(new SetWaistPosition(0));
    // m_operatorContoller.b().onTrue(new SetWaistPosition(180));
    // m_operatorContoller.x().onTrue(new ZeroWaistPosition());
    // m_operatorContoller.y().onTrue(new ZeroWaistPositionCoarse());

    // m_operatorContoller.leftBumper().onTrue(new SetArmExtension(0));
    // m_operatorContoller.rightBumper().onTrue(new SetArmExtension(0.5));
    
    // m_operatorContoller.x().onTrue(new ClawOpen());
    // m_operatorContoller.a().onTrue(new ClawClose());
    // m_operatorContoller.x().onTrue(new SetShoulderPosition(0));
    // m_operatorContoller.y().onTrue(new SetShoulderPosition(90));
    // m_operatorContoller.a().onTrue(new DriveShoulderWithJoystick(()->{return 0.0;}));
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

  @Log
  public static boolean isPracticeBot() {
    return !m_practiceBotJumper.get();
  }
}
