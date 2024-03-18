/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.commands

import edu.wpi.first.wpilibj2.command.Command
import org.hangar84.robot_2024.subsystems.DriveSubsystem

class DriveDistanceCommand(private val distanceInches: Double) : Command() {
    private val driveSubsystem = DriveSubsystem


    init {
        addRequirements(driveSubsystem)
    }

    override fun initialize() {
        driveSubsystem.leftEncoder.reset()
        driveSubsystem.rightEncoder.reset()

        driveSubsystem.leftPIDController.setpoint = distanceInches
        driveSubsystem.rightPIDController.setpoint = distanceInches
    }

    override fun execute() {
        val leftSpeed =
            driveSubsystem.leftPIDController.calculate(driveSubsystem.leftEncoder.distance).coerceIn(-1.0, 1.0)
        val rightSpeed =
            driveSubsystem.rightPIDController.calculate(driveSubsystem.rightEncoder.distance).coerceIn(-1.0, 1.0)

        driveSubsystem.differentialDrive.tankDrive(leftSpeed, rightSpeed)
    }

    override fun isFinished(): Boolean {
        return driveSubsystem.leftPIDController.atSetpoint() && driveSubsystem.rightPIDController.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.differentialDrive.stopMotor()
    }
}
