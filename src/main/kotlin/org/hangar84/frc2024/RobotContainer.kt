/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.hangar84.frc2024.subsystems.DriveSubsystem
import org.hangar84.frc2024.subsystems.LEDSubsystem
import org.hangar84.frc2024.subsystems.LauncherSubsystem
import org.hangar84.frc2024.wrappers.InvertibleCommandXboxController

object RobotContainer {
    private val controller = InvertibleCommandXboxController(0)
    private var autoChooser: SendableChooser<Command>? = null

    val autonomousCommand: Command
        get() {
            return autoChooser?.selected ?: InstantCommand()
        }

    init {
        // We disregard the returned UsbCamera instances, as we don't need them
        CameraServer.startAutomaticCapture("Front Camera", 0)
        CameraServer.startAutomaticCapture("Rear Camera", 1)

        configureBindings()
        configureNamedCommands()

        AutoBuilder.configureRamsete(
            { DriveSubsystem.pose },
            { pose: Pose2d -> DriveSubsystem.resetPose(pose) },
            { DriveSubsystem.chassisSpeeds },
            { speeds: ChassisSpeeds -> DriveSubsystem.driveRelative(speeds) },
            ReplanningConfig(),
            { DriverStation.getAlliance()?.get() == DriverStation.Alliance.Red },
            DriveSubsystem,
        )

        autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Autonomous Routine", autoChooser)

        LEDSubsystem.register()
    }

    /**
     * Method containing all controls and bindings for the robot.
     * ## Mappings:
     * ### Teleoperated Controls
     * - **Left Joystick (Y-Axis)**: Drive (Forward/Backward)
     * - **Right Joystick (X-Axis)**: Drive (Turn Left/Right)
     * - **Left Trigger**: Launcher (Intake)
     * - **Right Trigger**: Launcher (Launch)
     *
     * ### Characterization Controls
     * - **X** + **Left Bumper**: Run quasi-static characterization test (reverse)
     * - **X** + **Right Bumper**: Run quasi-static characterization test (forward)
     * - **Y** + **Left Bumper**: Run dynamic characterization test (reverse)
     * - **Y** + **Right Bumper**: Run dynamic characterization test (forward)
     */
    private fun configureBindings() {
        // Invert the Y-axis of the left joystick using our custom InvertibleCommandXboxController class.
        controller.yAxisInverted = true

        // -- Teleop-based default commands --

        /*
        Drive the robot using arcade drive.
        The left joystick Y-axis controls forward/backward movement, while the right joystick X-axis controls turning
        left/right.

        TODO: See if X-axis inversion is necessary.
         */
        DriveSubsystem.defaultCommand =
            DriveSubsystem.run {
                DriveSubsystem.arcadeDrive(controller.leftY, -controller.rightX)
            }

        /*
        Control the launcher using the left (intake/negative power) and right (launch/positive power) triggers.
         */
        LauncherSubsystem.defaultCommand =
            LauncherSubsystem.run {
                LauncherSubsystem.launcherMotor.set(-controller.leftTriggerAxis + controller.rightTriggerAxis)
            }

        // -- Characterization-based commands --
        controller
            .x()
            .and(controller.leftBumper())
            .and(Robot::isTestEnabled)
            .whileTrue(DriveSubsystem.getQuasistaticTestCommand(SysIdRoutine.Direction.kReverse))

        controller
            .x()
            .and(controller.rightBumper())
            .and(Robot::isTestEnabled)
            .whileTrue(DriveSubsystem.getQuasistaticTestCommand(SysIdRoutine.Direction.kForward))

        controller
            .y()
            .and(controller.leftBumper())
            .and(Robot::isTestEnabled)
            .whileTrue(DriveSubsystem.getDynamicTestCommand(SysIdRoutine.Direction.kReverse))

        controller
            .y()
            .and(controller.rightBumper())
            .and(Robot::isTestEnabled)
            .whileTrue(DriveSubsystem.getDynamicTestCommand(SysIdRoutine.Direction.kForward))
    }

    private fun configureNamedCommands() {
        // Launch the note game piece.
        NamedCommands.registerCommand(
            "Launch",
            LauncherSubsystem
                .runOnce {
                    LauncherSubsystem.launcherMotor.set(1.0)
                }
                .andThen(
                    WaitCommand(1.0),
                )
                .andThen(
                    LauncherSubsystem.runOnce {
                        LauncherSubsystem.launcherMotor.set(0.0)
                    },
                ),
        )

        // Take in the note game piece.
        NamedCommands.registerCommand(
            "Intake",
            LauncherSubsystem
                .runOnce {
                    LauncherSubsystem.launcherMotor.set(-1.0)
                }
                .andThen(
                    WaitCommand(1.0),
                )
                .andThen(
                    LauncherSubsystem.runOnce {
                        LauncherSubsystem.launcherMotor.set(0.0)
                    },
                ),
        )
    }
}
