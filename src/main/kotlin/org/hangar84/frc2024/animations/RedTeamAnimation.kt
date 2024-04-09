/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.animations

import org.hangar84.frc2024.animationsystem.*

object RedTeamAnimation : Animation(
    frames =
        listOf(
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                        ColorMap(ZiaComponents.RING_1, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_1, RGB.RED),
                        ColorMap(ZiaComponents.RING_2, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_2, RGB.RED),
                        ColorMap(ZiaComponents.RING_3, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_3, RGB.RED),
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.RED),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.RED),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.RED),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.RED),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.RED),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.RED),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.RED),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.RED),
                    ),
                frameLength = 0.25,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_3, RGB.RED),
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_2, RGB.RED),
                        ColorMap(ZiaComponents.RING_3, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.RING_1, RGB.RED),
                        ColorMap(ZiaComponents.RING_2, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                        ColorMap(ZiaComponents.RING_1, RGB.RED),
                    ),
                frameLength = 0.15,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING, RGB.RED),
                    ),
                frameLength = 0.15,
                frameGap = 0.15,
            ),
        ),
)
