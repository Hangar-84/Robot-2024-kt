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
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.abs

/**
 * Data table containing all network table entries for the drive subsystem.
 */
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
    // -- Constants --
    private const val METER_CONVERSION_FACTOR = 39.3701

    private const val TRACK_WIDTH = 21.8 / METER_CONVERSION_FACTOR
    private const val PULSES_PER_REVOLUTION = 2048
    private const val WHEEL_DIAMETER = 6.0 / METER_CONVERSION_FACTOR
    private const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI
    private const val GEAR_RATIO = 1.0 / 1.0
    private const val DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / (PULSES_PER_REVOLUTION * GEAR_RATIO)

    // -- Components --
    private val leftMotor = WPI_TalonSRX(0)
    private val rightMotor = WPI_TalonSRX(1)

    /**
     * The left follower motor, which follows the left motor's output.
     */
    private val leftFollowerMotor = WPI_VictorSPX(0)

    /**
     * The right follower motor, which follows the right motor's output.
     */
    private val rightFollowerMotor = WPI_VictorSPX(1)

    /**
     * The Inertial Measurement Unit (IMU) used to track the robot's rotation, acceleration, and orientation.
     *
     * @see ADIS16470_IMU
     */
    private val imu = ADIS16470_IMU()

    /**
     * The left encoder used to track the distance traveled by the left side of the robot.
     *
     * Encoder model: [Through Bore Encoder](https://www.revrobotics.com/rev-11-1271/)
     * @see Encoder
     */
    internal val leftEncoder = Encoder(0, 1, true)

    /**
     * The right encoder used to track the distance traveled by the right side of the robot.
     *
     * Encoder model: [Through Bore Encoder](https://www.revrobotics.com/rev-11-1271/)
     * @see Encoder
     */
    internal val rightEncoder = Encoder(2, 3, true)

    // -- Attributes --
    val differentialDrive = DifferentialDrive(leftMotor, rightMotor)

    /**
     * The odometry used to track the robot's position and orientation.
     * @see DifferentialDriveOdometry
     */
    private val differentialDriveOdometry =
        DifferentialDriveOdometry(
            Rotation2d.fromDegrees(imu.angle),
            leftEncoder.distance,
            rightEncoder.distance,
            // TODO: Configure initial pose(?)
            Pose2d(),
        )

    /**
     * The kinematics used to calculate the robot's wheel speeds based on the desired chassis speeds.
     * @see DifferentialDriveKinematics
     */
    private val differentialDriveKinematics = DifferentialDriveKinematics(TRACK_WIDTH)

    /**
     * The feedforward used to calculate the motor output based on the desired speed.
     *
     * See: [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html)
     * @see SimpleMotorFeedforward
     */
    private var driveFeedforward = SimpleMotorFeedforward(1.1, 2.0, 0.0)

    // TODO: Tune PID constants
    private const val DRIVE_P = 0.0
    private const val DRIVE_I = 0.0
    private const val DRIVE_D = 0.0

    /**
     * The PID controller used to adjust the left motor's output based on the desired speed.
     * @see PIDController
     */
    internal val leftPIDController = PIDController(DRIVE_P, DRIVE_I, DRIVE_D)

    /**
     * The PID controller used to adjust the right motor's output based on the desired speed.
     * @see PIDController
     */
    internal val rightPIDController = PIDController(DRIVE_P, DRIVE_I, DRIVE_D)

    /**
     * The current pose of the robot based on the differential drive odometry.
     *
     * @see DifferentialDriveOdometry
     * @see Pose2d
     */
    val pose: Pose2d
        get() = differentialDriveOdometry.poseMeters

    /**
     * The current rotation of the robot based on the IMU.
     *
     * @see ADIS16470_IMU.getAngle
     * @see Rotation2d
     */
    private val rotation2d
        get() = Rotation2d.fromDegrees(imu.angle)

    /**
     * The current speeds of the robot in meters per second.
     */
    val chassisSpeeds: ChassisSpeeds
        get() =
            differentialDriveKinematics.toChassisSpeeds(
                DifferentialDriveWheelSpeeds(leftEncoder.rate, rightEncoder.rate),
            )

    init {
        rightMotor.inverted = true
        rightFollowerMotor.inverted = true

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

        DataTable.staticGainEntry.setDouble(driveFeedforward.ks)
        DataTable.velocityGainEntry.setDouble(driveFeedforward.kv)
        DataTable.accelerationGainEntry.setDouble(driveFeedforward.ka)
    }

    override fun periodic() {
        differentialDriveOdometry.update(
            Rotation2d.fromDegrees(imu.angle),
            leftEncoder.distance,
            rightEncoder.distance,
        )

        DataTable.leftVoltageEntry.setDouble(leftMotor.motorOutputVoltage)
        DataTable.leftFollowerVoltageEntry.setDouble(leftFollowerMotor.motorOutputVoltage)
        DataTable.rightVoltageEntry.setDouble(rightMotor.motorOutputVoltage)
        DataTable.rightFollowerVoltageEntry.setDouble(rightFollowerMotor.motorOutputVoltage)

        DataTable.leftVelocityEntry.setDouble(leftEncoder.rate)
        DataTable.rightVelocityEntry.setDouble(rightEncoder.rate)

        val staticGain = DataTable.staticGainEntry.getDouble(driveFeedforward.ks)
        val velocityGain = DataTable.velocityGainEntry.getDouble(driveFeedforward.kv)
        val accelerationGain = DataTable.accelerationGainEntry.getDouble(driveFeedforward.ka)

        if (driveFeedforward.ks != staticGain || driveFeedforward.kv != velocityGain || driveFeedforward.ka != accelerationGain) {
            driveFeedforward = SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain)
        }
    }

    /**
     * Resets the robot's pose based on the provided [Pose2d].
     *
     * @param pose The new pose of the robot.
     */
    fun resetPose(pose: Pose2d) {
        differentialDriveOdometry.resetPosition(rotation2d, leftEncoder.distance, rightEncoder.distance, pose)
    }

    /**
     * Drives the robot using [ChassisSpeeds] relative to the robot's frame of reference.
     * This method uses [SimpleMotorFeedforward]s and [PIDController]s to adjust the motor output to reach the desired
     * speed.
     *
     * @param relativeSpeeds The desired speed of the robot.
     */
    fun driveRelative(relativeSpeeds: ChassisSpeeds) {
        val wheelSpeeds = differentialDriveKinematics.toWheelSpeeds(relativeSpeeds)

        val leftFed = driveFeedforward.calculate(wheelSpeeds.leftMetersPerSecond)
        val rightFed = driveFeedforward.calculate(wheelSpeeds.rightMetersPerSecond)

        val leftOutput = leftPIDController.calculate(leftEncoder.rate, wheelSpeeds.leftMetersPerSecond)
        val rightOutput = rightPIDController.calculate(rightEncoder.rate, wheelSpeeds.rightMetersPerSecond)

        leftMotor.setVoltage(leftFed + leftOutput)
        rightMotor.setVoltage(rightFed + rightOutput)
    }

    /**
     * Custom wrapper around [DifferentialDrive.arcadeDrive] to account for drift when the robot is moving
     * forward/backward.
     *
     * @param forwardSpeed The forward/backward speed of the robot.
     * @param rotationSpeed The rotation speed of the robot.
     * @see DifferentialDrive.arcadeDrive
     */
    fun arcadeDrive(
        forwardSpeed: Double,
        rotationSpeed: Double,
    ) {
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
