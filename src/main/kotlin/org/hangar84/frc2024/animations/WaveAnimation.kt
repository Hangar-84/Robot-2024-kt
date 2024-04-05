/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.animations

import org.hangar84.frc2024.animationsystem.*

object WaveAnimation : Animation(
    frames =
        listOf(
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.CYAN),
                    ),
                frameLength = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.CYAN),
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.BLUE),
                    ),
                frameLength = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.CYAN),
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_1, RGB.BLUE),
                    ),
                frameLength = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.CYAN),
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_1, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_2, RGB.BLUE),
                    ),
                frameLength = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.CYAN),
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_1, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_2, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_3, RGB.BLUE),
                    ),
                frameLength = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.CENTER_RING_EDGES, RGB.CYAN),
                        ColorMap(ZiaComponents.CENTER_RING_CORNERS, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_1, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_2, RGB.CYAN),
                        ColorMap(ZiaComponents.RING_3, RGB.CYAN),
                        ColorMap(ZiaComponents.ALL_TIPS, RGB.BLUE),
                    ),
                frameLength = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.CYAN),
                    ),
                frameLength = 1.0,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.CYAN),
                    ),
                frameLength = 0.5,
                frameGap = 0.25,
            ),
            AnimationFrame(
                buffer =
                    listOf(
                        ColorMap(ZiaComponents.ALL_PIXELS, RGB.CYAN),
                    ),
                frameLength = 0.5,
                frameGap = 1.0,
            ),
        ),
)
