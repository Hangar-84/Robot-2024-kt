/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.animationsystem

import edu.wpi.first.wpilibj.AddressableLEDBuffer

open class AnimationFrame(buffer: List<ColorMap>, val frameLength: Double, val frameGap: Double = 0.0) {
    val ledBuffer: AddressableLEDBuffer = AddressableLEDBuffer(ZiaComponents.LED_COUNT)

    init {
        buffer.forEach { colorMap ->
            colorMap.indices.forEach { index ->
                ledBuffer.setRGB(index, colorMap.color.r, colorMap.color.g, colorMap.color.b)
            }
        }
    }
}
