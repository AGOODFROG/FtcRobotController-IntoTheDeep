package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver
import org.firstinspires.ftc.teamcode.modular.HuskyLens.LensMode
import java.util.concurrent.TimeUnit

@Suppress("MemberVisibilityCanBePrivate")
abstract class BaseLinearOpMode : LinearOpMode() {
    protected lateinit var leftBack: DcMotorEx
    protected lateinit var rightBack: DcMotorEx
    protected lateinit var rightFront: DcMotorEx
    protected lateinit var leftFront: DcMotorEx
    protected lateinit var allMotors: Array<DcMotorEx>
    protected lateinit var odometry: GoBildaPinpointDriver
    protected lateinit var spinner: Spinner
    protected lateinit var bucket: Servo
    protected lateinit var switch: TouchSensor
    protected lateinit var elevator: DcMotorEx
    protected lateinit var arm: DcMotorEx

    protected lateinit var ratchet: ServoWrapper
    protected lateinit var hooks: ServoWrapper

    protected lateinit var huskyLens: HuskyLens


    // So having a ratelimit is apparently important, "to make it easier to read"

    //TODO move to initHarware
    protected var huckReadPeriod: Long = 1
    protected var huskyRateLimit: Deadline = Deadline(huckReadPeriod, TimeUnit.SECONDS)



    protected fun initHardware(unlatchRatchet: Boolean, /*mode: LensMode = LensMode.COLOR_RECOGNITION*/) {


        this.telemetry.msTransmissionInterval = 10

        this.hardwareMap

        //this.huskyLens = hardwareMap.get(HuskyLens::class.java, "husky_lens")
        this.huskyLens = this.hardwareMap["husky_lens"]  as HuskyLens

        this.leftBack = this.hardwareMap["left_back"] as DcMotorEx
        this.rightBack = this.hardwareMap["right_back"] as DcMotorEx
        this.rightFront = this.hardwareMap["right_front"] as DcMotorEx
        this.leftFront = this.hardwareMap["left_front"] as DcMotorEx

        this.leftFront.direction = DcMotorSimple.Direction.REVERSE
        this.rightFront.direction = DcMotorSimple.Direction.FORWARD
        this.leftBack.direction = DcMotorSimple.Direction.REVERSE
        this.rightBack.direction = DcMotorSimple.Direction.FORWARD

        this.allMotors = arrayOf(this.leftFront, this.rightFront, this.leftBack, this.rightBack)

        this.odometry = this.hardwareMap["odometry"] as GoBildaPinpointDriver
        this.odometry.setOffsets(95.0, 0.0)
        this.odometry.setEncoderResolution(37.25135125)
        this.odometry.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        )
        this.odometry.resetPosAndIMU()

        this.arm = this.hardwareMap["arm"] as DcMotorEx
        this.arm.direction = DcMotorSimple.Direction.REVERSE
        this.arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        this.arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        this.elevator = this.hardwareMap["elevator"] as DcMotorEx
        this.elevator.direction = DcMotorSimple.Direction.REVERSE
        this.elevator.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        this.ratchet = ServoWrapper(this.hardwareMap.servo["ratchet"], 0.12, 0.0, this)
        if (unlatchRatchet) this.ratchet.disengage() else this.ratchet.engage()

        this.hooks = ServoWrapper(this.hardwareMap.servo["hooks"], 0.6, 0.08, this)
        this.hooks.disengage()

        val leftSpinner = this.hardwareMap["left_spinner"] as CRServo
        leftSpinner.direction = DcMotorSimple.Direction.REVERSE
        val rightSpinner = this.hardwareMap["right_spinner"] as CRServo
        this.spinner = Spinner(leftSpinner, rightSpinner)

        this.bucket = this.hardwareMap["bucket"] as Servo

        this.switch = this.hardwareMap["touch_sensor"] as TouchSensor


        // todo uses hardware map
        //huskyLens = hardwareMap.get(HuskyLens::class.java, "huskylens")
        require(huskyLens.knock()) { "Failed to communicate with HuskyLens" }
        huskyLens.initialize()
        /*
        when (mode) {
            LensMode.TAG_RECOGNITION -> huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION)
            LensMode.COLOR_RECOGNITION -> huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION)
            LensMode.OBJECT_RECOGNITION -> huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION)
            LensMode.OBJECT_CLASSIFICATION -> huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION)
        }

         */


    }

}



