package org.firstinspires.ftc.teamcode.drive.robot

import androidx.annotation.NonNull
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil
import java.util.*
import kotlin.collections.ArrayList

@Config
class SampleMecanumDrive(hardwareMap: HardwareMap) : MecanumDrive(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER) {

    companion object {
        var TRANSLATIONAL_PID = PIDCoefficients(0.0, 0.0, 0.0)
        var HEADING_PID = PIDCoefficients(0.0, 0.0, 0.0)
        var LATERAL_MULTIPLIER = 1.0
        var VX_WEIGHT = 1.0
        var VY_WEIGHT = 1.0
        var OMEGA_WEIGHT = 1.0

        private val VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH)
        private val ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL)

        fun getVelocityConstraint(maxVel: Double, maxAngularVel: Double, trackWidth: Double): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(listOf(
                AngularVelocityConstraint(maxAngularVel),
                MecanumVelocityConstraint(maxVel, trackWidth)
            ))
        }

        fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(maxAccel)
        }
    }

    private var trajectorySequenceRunner: TrajectorySequenceRunner
    private var follower: TrajectoryFollower = HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID, Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5)

    private var leftFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftFront")
    private var leftRear: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftRear")
    private var rightRear: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rightRear")
    private var rightFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rightFront")

    private var motors = listOf(leftFront, leftRear, rightRear, rightFront)

    private var imu: IMU = hardwareMap.get(IMU::class.java, "imu")
    private var batteryVoltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()

    private var lastEncPositions: MutableList<Int> = ArrayList()
    private var lastEncVels: MutableList<Int> = ArrayList()

    init {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        val parameters = IMU.Parameters(RevHubOrientationOnRobot(DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR))
        imu.initialize(parameters)

        motors.forEach { motor ->
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID)
        }

        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftRear.direction = DcMotorSimple.Direction.REVERSE

        trajectorySequenceRunner = TrajectorySequenceRunner(follower, HEADING_PID, batteryVoltageSensor, lastEncPositions, lastEncVels, ArrayList(), ArrayList())
    }

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT, MAX_ANG_VEL, MAX_ANG_ACCEL)
    }

    fun turnAsync(angle: Double) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequenceBuilder(poseEstimate).turn(angle).build())
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequenceBuilder(trajectory.start()).addTrajectory(trajectory).build())
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    fun getLastError(): Pose2d {
        return trajectorySequenceRunner.lastPoseError
    }

    fun update() {
        updatePoseEstimate()
        val signal = trajectorySequenceRunner.update(poseEstimate, poseVelocity)
        if (signal != null) setDriveSignal(signal)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) update()
    }

    fun isBusy(): Boolean {
        return trajectorySequenceRunner.isBusy
    }

    fun setMode(runMode: DcMotor.RunMode) {
        motors.forEach { it.mode = runMode }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        motors.forEach { it.zeroPowerBehavior = zeroPowerBehavior }
    }

    fun setPIDFCoefficients(runMode: DcMotor.RunMode, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        motors.forEach { it.setPIDFCoefficients(runMode, compensatedCoefficients) }
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel = drivePower
        if (Math.abs(drivePower.x) + Math.abs(drivePower.y) + Math.abs(drivePower.heading) > 1) {
            val denom = VX_WEIGHT * Math.abs(drivePower.x) + VY_WEIGHT * Math.abs(drivePower.y) + OMEGA_WEIGHT * Math.abs(drivePower.heading)
            vel = Pose2d(VX_WEIGHT * drivePower.x, VY_WEIGHT * drivePower.y, OMEGA_WEIGHT * drivePower.heading).div(denom)
        }
        setDrivePower(vel)
    }

    @NonNull
    override fun getWheelPositions(): List<Double> {
        lastEncPositions.clear()
        return motors.map { motor ->
            val position = motor.currentPosition
            lastEncPositions.add(position)
            encoderTicksToInches(position)
        }
    }

    override fun getWheelVelocities(): List<Double> {
        lastEncVels.clear()
        return motors.map { motor ->
            val velocity = motor.velocity.toInt()
            lastEncVels.add(velocity)
            encoderTicksToInches(velocity)
        }
    }

    override fun setMotorPowers(v: Double, v1: Double, v2: Double, v3: Double) {
        leftFront.power = v
        leftRear.power = v1
        rightRear.power = v2
        rightFront.power = v3
    }

    override fun getRawExternalHeading(): Double {
        return imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
    }

    override fun getExternalHeading
