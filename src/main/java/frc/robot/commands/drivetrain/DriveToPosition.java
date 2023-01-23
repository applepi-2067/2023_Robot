// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.Math;

public class DriveToPosition extends CommandBase {
    private static Drivetrain m_driveTrain;
    private static double m_inches;
    private static double m_acceptableErrorInches = 0.1;

    public DriveToPosition(Drivetrain driveTrain, double inch) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
        m_driveTrain = driveTrain;
        m_inches = inch;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.setSetPointDistance(m_inches);
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
        double inchesError = m_inches - m_driveTrain.getDistanceTraveled();
        System.out.println(inchesError);

        return (Math.abs(inchesError) < m_acceptableErrorInches);
    }
}