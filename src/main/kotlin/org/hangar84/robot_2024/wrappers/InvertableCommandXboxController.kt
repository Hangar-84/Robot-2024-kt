/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.wrappers

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

class InvertableCommandXboxController(port: Int) : CommandXboxController(port) {
    var yAxisInverted = false


    private fun invertValue(value: Double): Double {
        return if (yAxisInverted) -value else value
    }

    override fun getLeftY(): Double {
        return invertValue(super.getLeftY())
    }

    override fun getRightY(): Double {
        return invertValue(super.getRightY())
    }
}