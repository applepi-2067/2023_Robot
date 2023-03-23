// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.auto.CenterStartRoutine;
import frc.robot.commands.auto.PickupAndScore;
import frc.robot.commands.lights.SetLightsColor;
import frc.robot.subsystems.Lights;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 * If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  private RobotSetupPosition m_position;
  private static SendableChooser<RobotSetupPosition> m_positionChooser;

  private Command m_autoCommand;

  private Drivetrain m_drivetrain = Drivetrain.getInstance();

  public enum RobotSetupPosition {
    CENTER,
    TOP,
    BOTTOM
  }

  public Command createAuto(RobotSetupPosition position) {
    switch (position) {
      case CENTER:
        return new CenterStartRoutine();
      case TOP:
        return new PickupAndScore();
      case BOTTOM:
        return new PickupAndScore();
      default:
        return new SetLightsColor(Lights.Color.YELLOW); // TODO: Default mobility auto?
    }
  }

  /**
   * This function is run when the robot is first started up
   * and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate the RobotContainer. Perform all our button bindings,
    // and put the autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Initialize Position Selector Choices
    positionSelectInit();
  }

  /**
   * This function is called every 20 ms, no matter the mode.
   * Use this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions,
   * but before LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Run the Scheduler.
    // Poll buttons, add newly-scheduled commands, run already-scheduled
    // commands, remove finished or interrupted commands,
    // and run subsystem periodic() methods.
    // This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    Logger.updateEntries();

    m_robotContainer.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Only allow pushing the robot around if we aren't on a real field
    if (DriverStation.isFMSAttached()) {
      m_robotContainer.setCoastEnabled(false);
    } else {
      m_robotContainer.setCoastEnabled(true);
    }
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putData("Position Chooser", m_positionChooser);
    m_position = (RobotSetupPosition) m_positionChooser.getSelected();
  }

  public void positionSelectInit() {
    m_positionChooser = new SendableChooser<RobotSetupPosition>();
    m_positionChooser.setDefaultOption("Top", RobotSetupPosition.TOP);
    m_positionChooser.addOption("Bottom", RobotSetupPosition.BOTTOM);
    m_positionChooser.addOption("Center", RobotSetupPosition.CENTER);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.setCoastEnabled(false);

    // Schedule the autonomous command
    if (m_position != null) { // TODO: Schedule mobility auto?
      Constants.ScoringInfo.initScoringInfo(m_position);
      m_drivetrain.setOdometryPose2d(Constants.ScoringInfo.initialPose2d);

      m_autoCommand = createAuto(m_position);
      m_autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.setCoastEnabled(false);
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

}
