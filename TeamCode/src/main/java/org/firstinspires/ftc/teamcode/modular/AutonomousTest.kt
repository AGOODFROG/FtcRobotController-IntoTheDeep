package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver
import org.firstinspires.ftc.teamcode.bucketWaiting
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

@Suppress("unused")
@Autonomous(name = "Autonomous Test", group = "Auto", preselectTeleOp = "DriveTrain")
class AutonomousTest : BaseLinearOpMode() {
    /*
    - move backwards to buckets, score
    - forward a tiny bit (cannot have wheel touching leftmost element), +90
    - toward until element, turn on pickup
    - backward to wall, -90, tiny bit back to buckets, score
    - do 2-4, except move a tiny bit more forward
    - the scoring sequence will need to pivot around left front in order to align with the buckets
     */
    private val turn: (Int) -> Stage = { pos -> Stage { Target(Mode.TURN, pos, it.position) } }
    private val run: (Int) -> Stage = { pos -> Stage { Target(Mode.RUN, pos, it.position) } }
    private val pivot: (Int) -> Stage = { pos -> Stage { Target(Mode.PIVOT_RB, pos, it.position) } }
    private val score = Stage {
        ratchet.disengage()
        sleep(500)
        arm.power = 1.0
        while (arm.currentPosition < 5500 && this.opModeIsActive()) {
//            if (arm.currentPosition > 4000) {
//                bucket.position = 0.0
//            }
            // wait to go up
        }
        // arm reaches top
        arm.power = 0.0
        bucket.position = 0.0
        // wait for element to drop
        sleep(2000)

        arm.power = -1.0
        bucket.position = bucketWaiting
        while (!switch.isPressed && this.opModeIsActive()) {
            // wait to go down
        }
        // arm reaches bottom
        arm.power = 0.0
        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        ratchet.engage()
        null
    }
    private val startLoad = Stage {
        elevator.power = 0.65
        spinner.on(true)
        null
    }
    private val endLoad = Stage {
        elevator.power = 0.0
        spinner.off()
        null
    }
    private val sleepStage: (Long) -> Stage = { time ->
        Stage {
            sleep(time)
            null
        }
    }
    private val correct = Stage {
        Target(
            Mode.TURN,
            90 - it.position.getHeading(AngleUnit.DEGREES).toInt(),
            it.position
        )
    }

    private val stages = listOf(
        // score preload
        run(-225),
        pivot(-40),
        score,
        // go to other
        turn(-47),
        startLoad,
        run(800),
        sleepStage(2000),
        endLoad,
//        correct,
        run(-800),
        turn(45),
        score,
        // ratchet is being silly
        sleepStage(500)
    )

    private var stage = stages[0]
    private var stageIdx = 0
    private var stopped = false

    private fun reset() {
        this.allMotors.forEach { it.power = 0.0 }
        this.stage.reset()
    }

    override fun runOpMode() {
        this.initHardware(false)
        allMotors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        if (!this.switch.isPressed || !ratchet.engaged()) {
            telemetry.addLine("arm not in correct state, do not start!")
            telemetry.update()
        }
        this.waitForStart()
        while (this.opModeIsActive() && !stopped) {
            this.odometry.update()
            this.telemetry.addData("stage", this.stage)

            if (!this.stage.started()) {
                this.stage.start(this)
                // only moves motors if target nonnull
                val target = this.stage.target()
                if (target != null) {
                    val sign = target.sign()
                    target.mode().mappings().zip(this.allMotors)
                        .forEach { (dir: Int, motor: DcMotorEx) ->
                            motor.power = target.mode().speed() * dir * sign
                        }
                    sleep(50)
                }
            }

//            sleep(100)

            val finished = this.stage.check(this.odometry.position)
            this.telemetry.addData("finished", finished)
            if (finished) {
                // allows reuse of stages
                reset()
                sleep(100)
                stageIdx++
                if (stageIdx < this.stages.size) {
                    this.stage = this.stages[stageIdx]
                } else {
                    stopped = true
                }
            }
            this.telemetry.update()
        }
    }

    private enum class Mode(val speed: Double, vararg val mappings: Int) {
        RUN(0.3, 1, 1, 1, 1),
        TURN(0.3, 1, -1, 1, -1),
        STRAFE(0.3, 1, -1, -1, 1),
        PIVOT_RB(0.5, 1, 0, 1, 0);

        fun speed(): Double = speed
        fun mappings(): IntArray = mappings
    }

    private class Stage(val targetGetter: (GoBildaPinpointDriver) -> Target?) {
        private var target: Target? = null
        private var started = false
        private var finished = false

        fun start(inst: AutonomousTest) {
            target = targetGetter(inst.odometry)
            if (target == null) finished = true
            started = true
        }

        fun check(pos: Pose2D): Boolean {
            if (!finished && /* if target nonnull, already finished */ target?.check(pos) == true) {
                finished = true
            }
            return finished
        }

        fun reset() {
            started = false
            finished = false
        }

        fun started() = started
        fun target() = target
    }

    private inner class Target(
        private val mode: Mode,
        private var target: Int,
        private val start: Pose2D
    ) {
        val startHeading = fix(start.getHeading(AngleUnit.DEGREES))
        var lastHeading = startHeading
        var absoluteTarget = 0
        val previousHeadings = mutableListOf<Double>()

        init {
//            if (mode == Mode.PIVOT_RB) {
//                target = -target
//            }
            if (mode == Mode.TURN || mode == Mode.PIVOT_RB) {
                assert(abs(target) < 360)
                absoluteTarget = (startHeading + target + 360).toInt() % 360
            }
        }

        fun fix(heading: Double): Double {
            return if (heading < 0) abs(heading)
            else (180 + abs(heading - 180)) % 360
        }

        fun mode() = mode
        fun sign() = target.sign

        fun check(pos: Pose2D): Boolean {
            return when (mode) {
                Mode.RUN, Mode.STRAFE -> {
                    val curr =
                        abs((pos.getX(DistanceUnit.MM) - start.getX(DistanceUnit.MM)).pow(2)) + abs(
                            (pos.getY(DistanceUnit.MM) - start.getY(DistanceUnit.MM)).pow(2)
                        )
                    telemetry.addData("current", curr)
                    curr >= target * target
                }

                Mode.TURN, Mode.PIVOT_RB -> {
                    val currentHeading = fix(pos.getHeading(AngleUnit.DEGREES))
                    if (lastHeading != 0.0 && currentHeading == 0.0) {
                        telemetry.addLine("help")
                    }
                    telemetry.addData("start", startHeading)
                    telemetry.addData("last", lastHeading)
                    telemetry.addData("curr", currentHeading)
                    telemetry.addData("target", absoluteTarget)
                    telemetry.addData("prev", previousHeadings)
                    if (lastHeading == currentHeading || abs(currentHeading - lastHeading) < 0.1) {
                        telemetry.addLine("stalling for new heading")
                        return false
                    }
                    if (target < 0 && ((lastHeading > absoluteTarget && currentHeading <= absoluteTarget) || (currentHeading in 340.0..360.0 && lastHeading in 0.0..20.0 && lastHeading > absoluteTarget && currentHeading - 360 <= absoluteTarget)) ||
                        (target > 0 && ((lastHeading < absoluteTarget && currentHeading >= absoluteTarget) || (currentHeading in 0.0..20.0 && lastHeading in 340.0..360.0 && lastHeading - 360 < absoluteTarget && currentHeading >= absoluteTarget)))
                    ) {
                        return true
                    } else {
                        previousHeadings.add(currentHeading)
                        lastHeading = currentHeading
                        return false
                    }
                }
            }
        }
    }
}
