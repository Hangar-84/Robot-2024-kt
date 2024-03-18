/**
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.hangar84.robot_2024.commands.DriveDistanceCommand
import org.hangar84.robot_2024.subsystems.DriveSubsystem
import org.hangar84.robot_2024.subsystems.LauncherSubsystem
import org.hangar84.robot_2024.wrappers.InvertableCommandXboxController


object RobotContainer {
    private val controller = InvertableCommandXboxController(0)

    val autonomousCommand: Command
        get() {
            return DriveDistanceCommand(12.0 * 5.0)
        }


    init {
        // We disregard the returned UsbCamera instances as we don't need them
        CameraServer.startAutomaticCapture("Front Camera", 0)
        CameraServer.startAutomaticCapture("Rear Camera", 1)

        configureBindings()
    }

    /**
     * Method containing all controls and bindings for the robot.
     * ## Mappings:
     * - **Left Joystick (Y-Axis)**: Drive (Forward/Backward)
     * - **Right Joystick (X-Axis)**: Drive (Turn Left/Right)
     * - **Left Trigger**: Launcher (Intake)
     * - **Right Trigger**: Launcher (Shoot)
     */
    private fun configureBindings() {
        controller.yAxisInverted = true

        // Teleop-based default commands
        DriveSubsystem.defaultCommand = Commands.run(
            { DriveSubsystem.arcadeDrive(controller.leftY, controller.rightX) },
            DriveSubsystem
        )

        LauncherSubsystem.defaultCommand = Commands.run(
            { LauncherSubsystem.launcherMotor.set(-controller.leftTriggerAxis + controller.rightTriggerAxis) },
            LauncherSubsystem
        )
    }
}