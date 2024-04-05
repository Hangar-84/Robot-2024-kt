/*
 * Robot 2024 (Kotlin Edition) — Kotlin version of our robot code for the 2023-2024 FRC season.
 * Copyright (C) 2024  Hangar 84
 */

package org.hangar84.frc2024.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.frc2024.Robot
import org.hangar84.frc2024.animations.*
import org.hangar84.frc2024.animationsystem.Animation
import org.hangar84.frc2024.animationsystem.ZiaComponents
import kotlin.jvm.optionals.getOrNull

object LEDSubsystem : SubsystemBase() {
    private val EMPTY_BUFFER = AddressableLEDBuffer(48)

    // -- Components --
    private val ziaLED = AddressableLED(0)
    private val frameGapTimer = Timer()
    private val frameTimeTimer = Timer()

    // -- Animations --
    private var lastAnimation: Animation? = null
    private val currentAnimation: Animation
        get() {
            if (!DriverStation.isDSAttached()) {
                return ClockAnimation
            } else if (DriverStation.getMatchTime() != -1.0 && Robot.isDisabled) {
                return EmergencyAlertAnimation
            } else if (Robot.isDisabled) {
                return WaveAnimation
            } else if (DriverStation.isEStopped()) {
                return EmergencyAlertAnimation
            } else if (Robot.isAutonomousEnabled) {
                return AutonomousEnabledAnimation
            } else if (Robot.isEnabled) {
                if (!DriverStation.isFMSAttached() && !Robot.isTest) {
                    return EnabledAnimation
                }

                return when (DriverStation.getAlliance().getOrNull()) {
                    DriverStation.Alliance.Red -> RedTeamAnimation
                    DriverStation.Alliance.Blue -> BlueTeamAnimation
                    else -> EnabledAnimation
                }
            }

            return EmergencyAlertAnimation
        }
    private var withinFrameGap = false

    init {
        ziaLED.setLength(ZiaComponents.LED_COUNT)
        ziaLED.setData(EMPTY_BUFFER)
        ziaLED.start()

        frameTimeTimer.start()
    }

    override fun periodic() {
        if (lastAnimation != currentAnimation) {
            currentAnimation.resetFrameIndex()
            lastAnimation = currentAnimation
        }

        if (frameTimeTimer.hasElapsed(currentAnimation.currentFrame.frameLength)) {
            frameTimeTimer.stop()
            frameTimeTimer.reset()

            withinFrameGap = currentAnimation.currentFrame.frameGap > 0
            if (!withinFrameGap) {
                currentAnimation.advanceFrame()
                frameTimeTimer.start()
                return
            }

            ziaLED.setData(EMPTY_BUFFER)

            frameGapTimer.start()
            return
        }

        if (withinFrameGap && frameGapTimer.hasElapsed(currentAnimation.currentFrame.frameGap)) {
            frameGapTimer.stop()
            frameGapTimer.reset()

            currentAnimation.advanceFrame()
            withinFrameGap = false

            frameTimeTimer.start()
        }

        if (withinFrameGap) {
            return
        }

        SmartDashboard.putNumber("Animation/Frame", currentAnimation.currentFrameIndex.toDouble())
        SmartDashboard.putNumber("Animation/Frame Length", currentAnimation.currentFrame.frameLength)
        SmartDashboard.putNumber("Animation/Frame Gap", currentAnimation.currentFrame.frameGap)
        ziaLED.setData(currentAnimation.currentFrame.ledBuffer)
    }
}
