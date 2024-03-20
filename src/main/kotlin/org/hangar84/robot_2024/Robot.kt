package org.hangar84.robot_2024

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {
    private var autonomousCommand: Command? = null


    override fun robotInit() {
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive_ArcadeStandard)
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_CommandControl)

        // Reference the RobotContainer object to ensure that the static initializer is run
        RobotContainer
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

    }

    override fun teleopInit() {
        autonomousCommand?.cancel()
    }

    override fun teleopPeriodic() {

    }

    override fun autonomousInit() {
        RobotContainer.autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {

    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {

    }

    override fun simulationInit() {

    }

    override fun simulationPeriodic() {

    }
}