// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.Math;

public class DriveToPosition extends CommandBase {
    private static Drivetrain m_driveTrain;
    private static double m_meters;
    private static double m_acceptableErrorMeters = 0.00254;

    public DriveToPosition(Drivetrain driveTrain, double meters) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
        m_driveTrain = driveTrain;
        m_meters = meters;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.setSetPointDistance(m_meters);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetain motors
        m_driveTrain.arcadeDrive(0, 0);
    }

    // Returns true when we are within an acceptable distance of our target position
    @Override
    public boolean isFinished() {
        double metersError = m_meters - m_driveTrain.getDistanceTraveled();
        System.out.println(metersError);

        return (Math.abs(metersError) < m_acceptableErrorMeters);
    }
}