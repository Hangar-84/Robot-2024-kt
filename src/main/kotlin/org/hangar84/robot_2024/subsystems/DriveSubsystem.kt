/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.abs

data object DataTable {
    private val table = NetworkTableInstance.getDefault().getTable("DriveData")

    val leftVelocityEntry: NetworkTableEntry = table.getEntry("Left Velocity")
    val rightVelocityEntry: NetworkTableEntry = table.getEntry("Right Velocity")

    val leftVoltageEntry: NetworkTableEntry = table.getEntry("Left Voltage")
    val leftFollowerVoltageEntry: NetworkTableEntry = table.getEntry("Left Follower Voltage")
    val rightVoltageEntry: NetworkTableEntry = table.getEntry("Right Voltage")
    val rightFollowerVoltageEntry: NetworkTableEntry = table.getEntry("Right Follower Voltage")

    val staticGainEntry: NetworkTableEntry = table.getEntry("Static Gain")
    val velocityGainEntry: NetworkTableEntry = table.getEntry("Velocity Gain")
    val accelerationGainEntry: NetworkTableEntry = table.getEntry("Acceleration Gain")
}

object DriveSubsystem : Subsystem {
    private const val METER_CONVERSION_FACTOR = 0.0254

    private const val TRACK_WIDTH = 21.8 * METER_CONVERSION_FACTOR
    private const val PULSES_PER_REVOLUTION = 8192.0
    private const val WHEEL_DIAMETER = 6.0 * METER_CONVERSION_FACTOR
    private const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI
    private const val GEAR_RATIO = 1.0 / 1.0
    private const val DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / (PULSES_PER_REVOLUTION * GEAR_RATIO)


    private val leftMotor = WPI_TalonSRX(0)
    private val rightMotor = WPI_TalonSRX(1)

    private val leftFollowerMotor = WPI_VictorSPX(0)
    private val rightFollowerMotor = WPI_VictorSPX(1)

    val differentialDrive = DifferentialDrive(leftMotor, rightMotor)

    // Inertial Measurement Unit
    private val imu = ADIS16470_IMU()

    internal val leftEncoder = Encoder(0, 1, true, CounterBase.EncodingType.k4X)
    internal val rightEncoder = Encoder(2, 3, false, CounterBase.EncodingType.k4X)


    private val differentialDriveOdometry = DifferentialDriveOdometry(
        Rotation2d.fromDegrees(imu.angle),
        leftEncoder.distance,
        rightEncoder.distance,
        // TODO: Configure initial pose(?)
        Pose2d()
    )

    // TODO: Measure track width
    private val differentialDriveKinematics = DifferentialDriveKinematics(TRACK_WIDTH)

    // See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
    private var leftFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)
    private var rightFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    private const val DRIVE_P = 0.05
    private const val DRIVE_I = 0.0
    private const val DRIVE_D = 0.0
    internal val leftPIDController = PIDController(DRIVE_P, DRIVE_I, DRIVE_D)
    internal val rightPIDController = PIDController(DRIVE_P, DRIVE_I, DRIVE_D)

    val pose: Pose2d
        get() = differentialDriveOdometry.poseMeters

    private val rotation2d
        get() = Rotation2d.fromDegrees(imu.angle)

    val chassisSpeeds
        get() = ChassisSpeeds(
            leftEncoder.rate,
            rightEncoder.rate,
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

        SmartDashboard.putData("Left PID Controller", leftPIDController)
        SmartDashboard.putData("Right PID Controller", rightPIDController)
        SmartDashboard.putData("Left Encoder", leftEncoder)
        SmartDashboard.putData("Right Encoder", rightEncoder)
        SmartDashboard.putData("IMU", imu)

        DataTable.staticGainEntry.setDouble(leftFeedforward.ks)
        DataTable.velocityGainEntry.setDouble(leftFeedforward.kv)
        DataTable.accelerationGainEntry.setDouble(leftFeedforward.ka)
    }

    override fun periodic() {
        differentialDriveOdometry.update(
            Rotation2d.fromDegrees(imu.angle),
            leftEncoder.distance,
            rightEncoder.distance
        )

        DataTable.leftVoltageEntry.setDouble(leftMotor.motorOutputVoltage)
        DataTable.leftFollowerVoltageEntry.setDouble(leftFollowerMotor.motorOutputVoltage)
        DataTable.rightVoltageEntry.setDouble(rightMotor.motorOutputVoltage)
        DataTable.rightFollowerVoltageEntry.setDouble(rightFollowerMotor.motorOutputVoltage)

        DataTable.leftVelocityEntry.setDouble(leftEncoder.rate)
        DataTable.rightVelocityEntry.setDouble(rightEncoder.rate)

        val staticGain = DataTable.staticGainEntry.getDouble(leftFeedforward.ks)
        val velocityGain = DataTable.velocityGainEntry.getDouble(leftFeedforward.kv)
        val accelerationGain = DataTable.accelerationGainEntry.getDouble(leftFeedforward.ka)

        if (leftFeedforward.ks != staticGain || leftFeedforward.kv != velocityGain || leftFeedforward.ka != accelerationGain)
            leftFeedforward = SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain)

        if (rightFeedforward.ks != staticGain || rightFeedforward.kv != velocityGain || rightFeedforward.ka != accelerationGain)
            rightFeedforward = SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain)
    }

    fun resetPose(pose: Pose2d) {
        differentialDriveOdometry.resetPosition(rotation2d, leftEncoder.distance, rightEncoder.distance, pose)
    }

    fun driveRelative(relativeSpeeds: ChassisSpeeds) {
        val wheelSpeeds = differentialDriveKinematics.toWheelSpeeds(relativeSpeeds)

        val leftFed = leftFeedforward.calculate(wheelSpeeds.leftMetersPerSecond)
        val rightFed = rightFeedforward.calculate(wheelSpeeds.rightMetersPerSecond)

        val leftOutput = leftPIDController.calculate(leftEncoder.rate, wheelSpeeds.leftMetersPerSecond)
        val rightOutput = rightPIDController.calculate(rightEncoder.rate, wheelSpeeds.rightMetersPerSecond)

        leftMotor.setVoltage(leftFed + leftOutput)
        rightMotor.setVoltage(rightFed + rightOutput)
    }

    fun arcadeDrive(forwardSpeed: Double, rotationSpeed: Double) {
        var zRotation = rotationSpeed

        if (abs(zRotation) < 0.15) {
            if (forwardSpeed > 0) {
                zRotation = 0.1
            } else if (forwardSpeed < 0) {
                zRotation = -0.1
            }
        }

        differentialDrive.arcadeDrive(forwardSpeed, zRotation)
    }
}