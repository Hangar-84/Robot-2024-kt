/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.animationsystem

/**
 * A simple data class representing the 8-bit red, green, and blue components of an LED color.
 */
data class RGB(val r: Int, val g: Int, val b: Int) {
    companion object {
        val BLACK = RGB(0, 0, 0)
        val WHITE = RGB(100, 100, 100)
        val RED = RGB(100, 0, 0)
        val GREEN = RGB(0, 100, 0)
        val BLUE = RGB(0, 0, 100)
        val YELLOW = RGB(100, 33, 0)
        val CYAN = RGB(0, 100, 100)
        val MAGENTA = RGB(100, 0, 100)

        val BRIGHT_WHITE = RGB(255, 255, 255)
        val BRIGHT_RED = RGB(255, 0, 0)
        val BRIGHT_BLUE = RGB(0, 0, 255)

        val AMBER = RGB(100, 20, 0)
    }
}
