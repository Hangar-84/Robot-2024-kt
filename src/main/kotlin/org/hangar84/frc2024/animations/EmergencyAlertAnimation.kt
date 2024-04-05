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
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.BLUE),
                    ),
                frameLength = 0.1,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.RED),
                    ),
                frameLength = 0.1,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.BLUE),
                    ),
                frameLength = 0.1,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.RED),
                    ),
                frameLength = 0.1,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.BLUE),
                    ),
                frameLength = 0.05,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.WHITE),
                        ColorMap(ZiaComponents.ALL_RAYS, RGB.RED),
                    ),
                frameLength = 0.1,
                frameGap = 0.1,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.WHITE),
                        ColorMap(ZiaComponents.ALL_RAYS, RGB.RED),
                    ),
                frameLength = 0.1,
                frameGap = 0.1,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.WHITE),
                        ColorMap(ZiaComponents.ALL_RAYS, RGB.RED),
                    ),
                frameLength = 0.1,
            ),
        ),
)
