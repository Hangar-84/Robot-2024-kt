/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.abs

object DriveSubsystem : Subsystem {
    private val leftMotor = WPI_TalonSRX(0)
    private val rightMotor = WPI_TalonSRX(1)

    private val leftFollowerMotor = WPI_VictorSPX(0)
    private val rightFollowerMotor = WPI_VictorSPX(1)

    val differentialDrive = DifferentialDrive(leftMotor, rightMotor)

    // TODO: Allow for drift correction to be configured
    private const val driftCorrectionDeadband = 0.1
    private const val driftCorrectionRotation = 0.1

    init {
        leftFollowerMotor.follow(leftMotor)
        rightFollowerMotor.follow(rightMotor)

        leftMotor.inverted = true
    }

    fun arcadeDrive(forwardSpeed: Double, rotationSpeed: Double) {
        var zRotation = rotationSpeed

        if (abs(forwardSpeed) < driftCorrectionDeadband) {
            if (forwardSpeed > 0.0) {
                zRotation = driftCorrectionRotation
            } else if (forwardSpeed < 0.0) {
                zRotation = -driftCorrectionRotation
            }
        }

        differentialDrive.arcadeDrive(forwardSpeed, zRotation)
    }
}