/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.animations

import org.hangar84.robot_2024.animationsystem.*

object ClockAnimation : Animation(
    frames =
        listOf(
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.TOP_RAY_2, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.TOP_RAY_1, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.RIGHT_RAY_3, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.RIGHT_RAY_2, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.RIGHT_RAY_1, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.BOTTOM_RAY_3, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.BOTTOM_RAY_2, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.BOTTOM_RAY_1, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.LEFT_RAY_3, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.LEFT_RAY_2, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.LEFT_RAY_1, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.TOP_RAY_3, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
        ),
)
