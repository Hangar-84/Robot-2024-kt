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

object EnabledAnimation : Animation(
    frames =
        listOf(
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.RING_1, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_1, RGB.GREEN),
                        ColorMap(ZiaComponents.RING_2, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_2, RGB.GREEN),
                        ColorMap(ZiaComponents.RING_3, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_3, RGB.GREEN),
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.GREEN),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.GREEN),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.GREEN),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.GREEN),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.GREEN),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.GREEN),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.GREEN),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.GREEN),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_3, RGB.GREEN),
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_2, RGB.GREEN),
                        ColorMap(ZiaComponents.RING_3, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_1, RGB.GREEN),
                        ColorMap(ZiaComponents.RING_2, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                        ColorMap(ZiaComponents.RING_1, RGB.GREEN),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.GREEN),
                    ),
                frameLength = 0.15,
                frameGap = 0.15,
            ),
        ),
)
