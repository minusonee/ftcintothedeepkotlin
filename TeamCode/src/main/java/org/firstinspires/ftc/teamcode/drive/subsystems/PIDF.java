package org.firstinspires.ftc.teamcode.drive.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp
public class PIDF extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 536 / 360;

    private DcMotorEx motorCraneLeft,motorCraneRight;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        motorCraneLeft = hardwareMap.get(DcMotorEx.class, "motorCraneLeft");
        motorCraneRight = hardwareMap.get(DcMotorEx.class, "motorCraneRight");

        motorCraneLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCraneLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCraneLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCraneRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCraneRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCraneRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(){
        controller.setPID(p, i, d);
        int armPos = motorCraneLeft.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        motorCraneLeft.setPower(power);
        motorCraneRight.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.addData("Power ", power);
        telemetry.update();
    }
}
