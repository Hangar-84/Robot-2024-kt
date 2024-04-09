/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.animations

import org.hangar84.frc2024.animationsystem.*

object EmergencyAlertAnimation : Animation(
    frames =
        listOf(
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.BRIGHT_BLUE),
                    ),
                frameLength = 0.075,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.BRIGHT_RED),
                    ),
                frameLength = 0.075,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.BRIGHT_BLUE),
                    ),
                frameLength = 0.075,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.BRIGHT_RED),
                    ),
                frameLength = 0.075,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.BRIGHT_BLUE),
                    ),
                frameLength = 0.075,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.BRIGHT_WHITE),
                        ColorMap(ZiaComponents.ALL_RAYS, RGB.BRIGHT_RED),
                    ),
                frameLength = 0.075,
                frameGap = 0.075,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.BRIGHT_WHITE),
                        ColorMap(ZiaComponents.ALL_RAYS, RGB.BRIGHT_RED),
                    ),
                frameLength = 0.075,
                frameGap = 0.075,
            ),
            AnimationFrame(
                buffer =
                listOf(
                    ColorMap(ZiaComponents.CENTER_RING, RGB.BRIGHT_WHITE),
                    ColorMap(ZiaComponents.ALL_RAYS, RGB.BRIGHT_RED),
                ),
                frameLength = 0.075,
                frameGap = 0.075,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.BRIGHT_WHITE),
                        ColorMap(ZiaComponents.ALL_RAYS, RGB.BRIGHT_RED),
                    ),
                frameLength = 0.075,
            ),
        ),
)
