/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.abs

object DriveSubsystem : Subsystem {
    private val leftMotor = WPI_TalonSRX(0)
    private val rightMotor = WPI_TalonSRX(1)

    private val leftFollowerMotor = WPI_VictorSPX(0)
    private val rightFollowerMotor = WPI_VictorSPX(1)

    val differentialDrive = DifferentialDrive(leftMotor, rightMotor)

    // Inertial Measurement Unit
    private val imu = ADIS16470_IMU()

    internal val leftEncoder = Encoder(0, 1, true, CounterBase.EncodingType.k4X)
    internal val rightEncoder = Encoder(2, 3, false, CounterBase.EncodingType.k4X)

    // TODO: Configure PID values
    private const val DRIVE_P = 0.0
    private const val DRIVE_I = 0.0
    private const val DRIVE_D = 0.0

    internal val leftPIDController = PIDController(DRIVE_P, DRIVE_I, DRIVE_D)
    internal val rightPIDController = PIDController(DRIVE_P, DRIVE_I, DRIVE_D)

    private const val PULSES_PER_REVOLUTION = 8192.0
    private const val WHEEL_DIAMETER = 6.0
    private const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI
    private const val GEAR_RATIO = 8.46 / 1.0
    private const val DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / (PULSES_PER_REVOLUTION * GEAR_RATIO)

    // TODO: Tune these values
    private val driftCorrectionPID = PIDController(0.0, 0.0, 0.0)
    private const val ROTATION_DEADBAND = 0.075

    init {
        leftMotor.inverted = true

        leftFollowerMotor.follow(leftMotor)
        rightFollowerMotor.follow(rightMotor)

        leftEncoder.distancePerPulse = DISTANCE_PER_PULSE
        rightEncoder.distancePerPulse = DISTANCE_PER_PULSE

        leftEncoder.reset()
        rightEncoder.reset()
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