/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.Subsystem

object LauncherSubsystem : Subsystem {
    val launcherMotor = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushed)
    private val launcherFollowerMotor = CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushed)

    init {
        launcherFollowerMotor.follow(launcherMotor)
        launcherMotor.inverted = true
    }
}
