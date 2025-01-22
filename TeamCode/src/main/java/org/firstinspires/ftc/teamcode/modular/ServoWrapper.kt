package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.robotcore.hardware.Servo

class ServoWrapper(private val servo: Servo, private val active: Double, private val inactive: Double) {
    private var engaged = false
    fun engaged() = engaged
    private var manual = false
    fun manual() = manual

    fun engage() {
        engaged = true
        servo.position = active
        Thread.sleep(500)
    }

    fun disengage() {
        engaged = false
        servo.position = inactive
        Thread.sleep(500)
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

    fun disablePwm() = servo.controller.pwmDisable()

    val position: Double
        get() = servo.position
}
