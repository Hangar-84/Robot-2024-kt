/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.animations

import org.hangar84.frc2024.animationsystem.Animation
import org.hangar84.frc2024.animationsystem.AnimationFrame
import org.hangar84.frc2024.animationsystem.ColorMap
import org.hangar84.frc2024.animationsystem.RGB
import org.hangar84.frc2024.animationsystem.ZiaComponents

object AutonomousEnabledAnimation : Animation(
    frames =
        listOf(
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.RED),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.RED),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.RED),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.AMBER),
                    ),
                frameLength = 0.10,
                frameGap = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                        ColorMap(ZiaComponents.RING_1, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_1, RGB.AMBER),
                        ColorMap(ZiaComponents.RING_2, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                        ColorMap(ZiaComponents.RING_2, RGB.AMBER),
                        ColorMap(ZiaComponents.RING_3, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_3, RGB.AMBER),
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.AMBER),
                    ),
                frameLength = 0.10,
                frameGap = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_3, RGB.AMBER),
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                        ColorMap(ZiaComponents.RING_2, RGB.AMBER),
                        ColorMap(ZiaComponents.RING_3, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_1, RGB.AMBER),
                        ColorMap(ZiaComponents.RING_2, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                        ColorMap(ZiaComponents.RING_1, RGB.AMBER),
                    ),
                frameLength = 0.10,
            ),
        ),
)
