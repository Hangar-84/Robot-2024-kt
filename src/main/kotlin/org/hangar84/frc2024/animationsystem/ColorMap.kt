/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.animationsystem

/**
 * A color map that maps LED strip indices to colors.
 */
data class ColorMap(
    val indices: Set<Int>,
    val color: RGB,
)
