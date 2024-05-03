/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.animationsystem

/**
 * Represents an entire animation, consisting of multiple frames.
 * @param frames The frames that make up the animation.
 * @property currentFrameIndex The index of the current frame.
 * @property currentFrame The current frame.
 * @see AnimationFrame
 */
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
