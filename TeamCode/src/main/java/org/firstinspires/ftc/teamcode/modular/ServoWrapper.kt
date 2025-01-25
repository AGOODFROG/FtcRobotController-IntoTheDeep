package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.robotcore.hardware.Servo
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.TimeSource

class ServoWrapper(
    private val servo: Servo,
    private val active: Double,
    private val inactive: Double
) {
    // must either engage or disengage at startup so it's in a known state
    private var engaged = false
    fun engaged() = engaged
    private var manual = false
    fun manual() = manual
    var blockUntil = TimeSource.Monotonic.markNow().minus(1.seconds)

    fun updateTime() {
        blockUntil = TimeSource.Monotonic.markNow().plus(500.milliseconds)
    }

    fun engage() {
        if (blockUntil > TimeSource.Monotonic.markNow()) return
        engaged = true
        servo.position = active
        updateTime()
    }

    fun disengage() {
        if (blockUntil > TimeSource.Monotonic.markNow()) return
        engaged = false
        servo.position = inactive
        updateTime()
    }

    fun enableManual() {
        manual = true
    }

    fun disableManual() {
        manual = false
    }

    fun toggle() {
        when (engaged) {
            true -> disengage()
            false -> engage()
        }
    }

    fun waiting() = blockUntil > TimeSource.Monotonic.markNow()

    fun disablePwm() = servo.controller.pwmDisable()

    val position: Double
        get() = servo.position
}
