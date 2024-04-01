/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.animationsystem

data class RGB(val r: Int, val g: Int, val b: Int) {
    companion object {
        val BLACK = RGB(0, 0, 0)
        val WHITE = RGB(255, 255, 255)
        val RED = RGB(255, 0, 0)
        val GREEN = RGB(0, 255, 0)
        val BLUE = RGB(0, 0, 255)
        val YELLOW = RGB(255, 255, 0)
        val CYAN = RGB(0, 255, 255)
        val MAGENTA = RGB(255, 0, 255)

        val AMBER = RGB(100, 75, 0)
    }
}
