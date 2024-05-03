/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.wrappers

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

/**
 * A wrapper for the XboxController that allows inverting the axes.
 *
 * @param port The port index on the Driver Station that the controller is plugged into.
 */
class InvertibleCommandXboxController(port: Int) : CommandXboxController(port) {
    var xAxisInverted = false
    var yAxisInverted = false

    override fun getLeftX(): Double {
        return if (xAxisInverted) -super.getLeftX() else super.getLeftX()
    }

    override fun getRightX(): Double {
        return if (xAxisInverted) -super.getRightX() else super.getRightX()
    }

    override fun getLeftY(): Double {
        return if (yAxisInverted) -super.getLeftY() else super.getLeftY()
    }

    override fun getRightY(): Double {
        return if (yAxisInverted) -super.getRightY() else super.getRightY()
    }
}
