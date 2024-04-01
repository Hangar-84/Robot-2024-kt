/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.animationsystem

object ZiaComponents {
    const val LED_COUNT = 48

    val ALL_PIXELS = (0 until LED_COUNT).toSet()

    // -- Center Circle Fragments --
    val CENTER_RING = (0..7).toSet()

    val CENTER_RING_CORNER_BOTTOM_RIGHT = setOf(0)
    val CENTER_RING_CORNER_BOTTOM_LEFT = setOf(2)
    val CENTER_RING_CORNER_TOP_LEFT = setOf(4)
    val CENTER_RING_CORNER_TOP_RIGHT = setOf(6)

    val CENTER_RING_BOTTOM_EDGE = setOf(1)
    val CENTER_RING_LEFT_EDGE = setOf(3)
    val CENTER_RING_TOP_EDGE = setOf(5)
    val CENTER_RING_RIGHT_EDGE = setOf(7)

    val CENTER_RING_CORNERS =
        CENTER_RING_CORNER_BOTTOM_RIGHT + CENTER_RING_CORNER_BOTTOM_LEFT + CENTER_RING_CORNER_TOP_LEFT + CENTER_RING_CORNER_TOP_RIGHT
    val CENTER_RING_EDGES =
        CENTER_RING_BOTTOM_EDGE + CENTER_RING_LEFT_EDGE + CENTER_RING_TOP_EDGE + CENTER_RING_RIGHT_EDGE

    // -- Individual Rays --
    val RIGHT_RAY_1 = (8..10).toSet()
    val RIGHT_RAY_2 = (11..14).toSet()
    val RIGHT_RAY_3 = (15..17).toSet()

    val TOP_RAY_1 = (18..20).toSet()
    val TOP_RAY_2 = (21..24).toSet()
    val TOP_RAY_3 = (25..27).toSet()

    val LEFT_RAY_1 = (28..30).toSet()
    val LEFT_RAY_2 = (31..34).toSet()
    val LEFT_RAY_3 = (35..37).toSet()

    val BOTTOM_RAY_1 = (38..40).toSet()
    val BOTTOM_RAY_2 = (41..44).toSet()
    val BOTTOM_RAY_3 = (45..47).toSet()

    // -- Ray Sets --
    val RIGHT_RAYS = RIGHT_RAY_1 + RIGHT_RAY_2 + RIGHT_RAY_3
    val TOP_RAYS = TOP_RAY_1 + TOP_RAY_2 + TOP_RAY_3
    val LEFT_RAYS = LEFT_RAY_1 + LEFT_RAY_2 + LEFT_RAY_3
    val BOTTOM_RAYS = BOTTOM_RAY_1 + BOTTOM_RAY_2 + BOTTOM_RAY_3

    val ALL_RAYS = RIGHT_RAYS + TOP_RAYS + LEFT_RAYS + BOTTOM_RAYS

    // -- Waves --
    val RIGHT_WAVE_1 = setOf(8, 11, 15)
    val RIGHT_WAVE_2 = setOf(9, 12, 16)
    val RIGHT_WAVE_3 = setOf(10, 13, 17)

    val TOP_WAVE_1 = setOf(18, 21, 25)
    val TOP_WAVE_2 = setOf(19, 22, 26)
    val TOP_WAVE_3 = setOf(20, 23, 27)

    val LEFT_WAVE_1 = setOf(28, 31, 35)
    val LEFT_WAVE_2 = setOf(29, 32, 36)
    val LEFT_WAVE_3 = setOf(30, 33, 37)

    val BOTTOM_WAVE_1 = setOf(38, 41, 45)
    val BOTTOM_WAVE_2 = setOf(39, 42, 46)
    val BOTTOM_WAVE_3 = setOf(40, 43, 47)

    // -- Rings --
    val RING_1 = RIGHT_WAVE_1 + TOP_WAVE_1 + LEFT_WAVE_1 + BOTTOM_WAVE_1
    val RING_2 = RIGHT_WAVE_2 + TOP_WAVE_2 + LEFT_WAVE_2 + BOTTOM_WAVE_2
    val RING_3 = RIGHT_WAVE_3 + TOP_WAVE_3 + LEFT_WAVE_3 + BOTTOM_WAVE_3

    // -- Quadrant Tips
    val RIGHT_TIP = setOf(14)
    val TOP_TIP = setOf(24)
    val LEFT_TIP = setOf(34)
    val BOTTOM_TIP = setOf(44)

    val ALL_TIPS = RIGHT_TIP + TOP_TIP + LEFT_TIP + BOTTOM_TIP
}
