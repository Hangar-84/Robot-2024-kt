/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.robot_2024.animationsystem

open class Animation(private val frames: List<AnimationFrame>) {
    var currentFrameIndex = 0
        private set

    val currentFrame
        get() = frames[currentFrameIndex]

    fun advanceFrame() {
        currentFrameIndex = (currentFrameIndex + 1) % frames.size
    }

    fun resetFrameIndex() {
        currentFrameIndex = 0
    }
}
