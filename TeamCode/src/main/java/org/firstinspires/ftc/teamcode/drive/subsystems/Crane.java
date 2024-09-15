package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Crane {
    //MOTOR DECLARATION
    public DcMotor motorCrane1, motorCrane2;
    public Servo servoGrippy, servoSlide1, servoSlide2;

    //PID VARIABLES
    public int craneTarget = 0;
    public static double craneP = 0.02, craneI = 0, craneD = 0.001;
    double craneIntegralSum = 0;
    public static double craneF = 0.25;
    public static int craneOffset = 50;
    private final double threetwelvemotorticksindegree = 537.7 / 360;
    double craneLastError = 0;
    ElapsedTime craneTimer = new ElapsedTime();

    //MOTOR INIT
    public Crane(HardwareMap hardwareMap){
        motorCrane1 = hardwareMap.dcMotor.get("motorCrane1");
        motorCrane2 = hardwareMap.dcMotor.get("motorCrane2");

        servoGrippy = hardwareMap.servo.get("servoGrippy");
        servoSlide1 = hardwareMap.servo.get("servoSlide1");
        servoSlide2 = hardwareMap.servo.get("servoSlide2");

        motorCrane1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCrane1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCrane1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCrane1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCrane2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCrane2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCrane2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCrane2.setDirection(DcMotorSimple.Direction.REVERSE);

        servoGrippy.setDirection(Servo.Direction.FORWARD);
        servoSlide1.setDirection(Servo.Direction.FORWARD);
        servoSlide2.setDirection(Servo.Direction.REVERSE);

    }
    //MOVE THE SLIDES FUNCTION
    public void moveSlides(double pos){
        servoSlide1.setPosition(pos);
        servoSlide2.setPosition(pos);
    }

    //SET THE POSITION OF THE CLAW FUNCTION
    public void setGripper(double pos){
        servoGrippy.setPosition(pos);
    }

    //PID FOR MOVING THE CRANE
    public double PIDControlCrane(double craneTarget){

        double cranePos = motorCrane1.getCurrentPosition();
        double craneError = craneTarget - cranePos;
        craneIntegralSum += craneError * craneTimer.seconds();
        double craneDerivative = (craneError - craneLastError) / craneTimer.seconds();
        craneLastError = craneError;

        craneTimer.reset();

        double craneOutput = (craneError * craneP) + (craneDerivative * craneD) + (craneIntegralSum * craneI);
        return craneOutput;
    }

    //RETURNS THE POWER CALCULATED BY THE PID
    public double cranePower(double craneTarget){
        double cranePos = motorCrane1.getCurrentPosition();
        double craneFF = Math.cos(Math.toRadians((cranePos - craneOffset) / threetwelvemotorticksindegree)) * craneF;
        double cranePower = PIDControlCrane(craneTarget) + craneFF;
        return cranePower;
    }



}


