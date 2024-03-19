/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
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
    private val rotation2d = Rotation2d.fromDegrees(imu.angle)

    internal val leftEncoder = Encoder(0, 1, true, CounterBase.EncodingType.k4X)
    internal val rightEncoder = Encoder(2, 3, false, CounterBase.EncodingType.k4X)

    // TODO: Tune drive PID values
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
    private const val METER_CONVERSION_FACTOR = 0.0254
    private const val INCH_CONVERSION_FACTOR = 1.0 / METER_CONVERSION_FACTOR

    // TODO: Tune drift correction PID values and/or deadband
    private val driftCorrectionPID = PIDController(0.0, 0.0, 0.0)
    private const val ROTATION_DEADBAND = 0.075

    private val differentialDriveOdometry = DifferentialDriveOdometry(
        Rotation2d.fromDegrees(imu.angle),
        leftEncoder.distance,
        rightEncoder.distance,
        // TODO: Configure initial pose(?)
        Pose2d()
    )

    // TODO: Measure track width
    private val differentialDriveKinematics = DifferentialDriveKinematics(21.8 * METER_CONVERSION_FACTOR)

    val pose: Pose2d
        get() = differentialDriveOdometry.poseMeters

    val chassisSpeeds
        get() = ChassisSpeeds(
            leftEncoder.rate * METER_CONVERSION_FACTOR,
            rightEncoder.rate * METER_CONVERSION_FACTOR,
            Math.toRadians(imu.rate)
        )

    init {
        leftMotor.inverted = true

        leftFollowerMotor.follow(leftMotor)
        rightFollowerMotor.follow(rightMotor)

        leftEncoder.distancePerPulse = DISTANCE_PER_PULSE
        rightEncoder.distancePerPulse = DISTANCE_PER_PULSE

        leftEncoder.reset()
        rightEncoder.reset()
    }

    override fun periodic() {
        differentialDriveOdometry.update(
            Rotation2d.fromDegrees(imu.angle),
            leftEncoder.distance * METER_CONVERSION_FACTOR,
            rightEncoder.distance * METER_CONVERSION_FACTOR
        )
    }

    fun resetPose(pose: Pose2d) {
        differentialDriveOdometry.resetPosition(rotation2d, leftEncoder.distance, rightEncoder.distance, pose)
    }

    fun driveRelative(relativeSpeeds: ChassisSpeeds) {
        val wheelSpeeds = differentialDriveKinematics.toWheelSpeeds(relativeSpeeds)

        leftPIDController.setpoint = wheelSpeeds.leftMetersPerSecond * INCH_CONVERSION_FACTOR
        rightPIDController.setpoint = wheelSpeeds.rightMetersPerSecond * INCH_CONVERSION_FACTOR

        val leftOutput = leftPIDController.calculate(leftEncoder.rate).coerceIn(-1.0, 1.0)
        val rightOutput = rightPIDController.calculate(rightEncoder.rate).coerceIn(-1.0, 1.0)

        differentialDrive.tankDrive(leftOutput, rightOutput)
    }

    fun arcadeDrive(forwardSpeed: Double, rotationSpeed: Double) {
        var zRotation = rotationSpeed

        if (abs(zRotation) < ROTATION_DEADBAND) {
            val distanceError = leftEncoder.distance - rightEncoder.distance
            zRotation = driftCorrectionPID.calculate(distanceError).coerceIn(-1.0, 1.0)
        }

        differentialDrive.arcadeDrive(forwardSpeed, zRotation)
    }
}