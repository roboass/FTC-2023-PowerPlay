package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class UsefulFunctions extends LinearOpMode {
    public DcMotor frontleft, frontright, backleft, backright;
    public DcMotor rampaMotorDreapta, rampaMotorStanga;

//    public Servo trafaletServoStanga, trafaletServoDreapta;
//    public DcMotor trafaletMotor;

    public Servo rampaServoStanga, rampaServoDreapta;
    public double rampaAngle;
//    public Servo launchServo, liftClawServo1, liftClawServo2, grabClawServo1, grabClawServo2, angleLaunchServo1, angleLaunchServo2;
//    public OpenCvCamera phoneCam;

    OpenCvCamera webcam;
    public OpenCvPipeline pipeline = new OpenCvPipeline() {
        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    };

    public static double ticks_rev = 55.1;//753.2, 145.6;
    public static double gear_ratio = 5.2;
    public static int diameter_mm = 100;
    public static double diameter_in = 3.94;

    public static double gearRatioOffset = 32/20; ///32 dde dinti pe 20 de dinti pentru ca un servo avea dinti diferite (servo dreapta rampa)
    // public static int trafaletPozJos = 80;

    // public double unghiNivelJos = 82.5, unghiNivelMij = 97.5, unghiNivelSus = 110, unghiCarousel = 100;
    public double unghiNivelJos = 45, unghiNivelMij = 55, unghiNivelSus = 65, unghiCarousel = 52.5;
    public double unghiNivelIntake = 20;

    public double unghiuriRampa[] = {unghiNivelJos, unghiNivelMij, unghiNivelSus};
    public static int rampaState = 0; /// 0 - JOS, 1 - MIJLOC, 2 - SUS

    public static boolean mergeRampa = false;

    public int crticksfl, crticksfr, crticksbl, crticksbr;

    public BNO055IMU gyro;
    public Orientation crtangle = new Orientation();

    public void Initialise() {
        frontleft = hardwareMap.get(DcMotor.class, "front_left");
        frontright = hardwareMap.get(DcMotor.class, "front_right");
        backleft = hardwareMap.get(DcMotor.class, "back_left");
        backright = hardwareMap.get(DcMotor.class, "back_right");
        rampaMotorDreapta = hardwareMap.get(DcMotor.class, "r_m_d");
        rampaMotorStanga = hardwareMap.get(DcMotor.class, "r_m_s");

        // trafaletMotor = hardwareMap.get(DcMotor.class, "trafalet");
        // trafaletServoStanga = hardwareMap.get(Servo.class, "trafalet_servo_st");
        // trafaletServoDreapta = hardwareMap.get(Servo.class, "trafalet_servo_dr");
        rampaServoStanga = hardwareMap.get(Servo.class, "rampa_servo_st");
        rampaServoDreapta = hardwareMap.get(Servo.class, "rampa_servo_dr");

        SwitchMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        rampaMotorDreapta.setDirection(DcMotorSimple.Direction.FORWARD);
        rampaMotorStanga.setDirection(DcMotorSimple.Direction.FORWARD);

//        trafaletMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        trafaletServoStanga.setDirection(Servo.Direction.REVERSE);
//        trafaletServoDreapta.setDirection(Servo.Direction.FORWARD);
//        trafaletAngle = 0;

        rampaServoStanga.setDirection(Servo.Direction.REVERSE);
        rampaServoDreapta.setDirection(Servo.Direction.REVERSE);

        //Partea drepta mere in fata
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        while (!gyro.isGyroCalibrated() && opModeIsActive())
        {
            telemetry.addData("IMU is calibrating!", "Please wait.");
            telemetry.update();
        }

        telemetry.addData("Init is done", "Press start");
        telemetry.update();
    }

    /*Asemanator cu functiile MoveSideMM si MoveFWBKMM merge-uite.
     * Ia ca parametri cat sa se miste pe axa x SAU pe axa y (in mm).
     * Una din axe trebe sa fie 0 altfel nu stiu ce se intampla
     */
    public void AutonomousMove(double x_mm, double y_mm) {
        double motorPower = 1;
        int sideOrFront;

        int ticksToMove_x = mm_to_ticks(x_mm);
        int ticksToMove_y = mm_to_ticks(y_mm);
        int ticksToMove = (ticksToMove_x == 0 ? ticksToMove_y : ticksToMove_x);
//        sideOrFront = (x_mm != 0 ? -1 : 1);
//        UpdateTicks();
//        int trgtfl, trgtfr, trgtbl, trgtbr;
//
//        trgtfl = crticksfl - ticksToMove;
//        trgtfr = crticksfr + sideOrFront * ticksToMove;
//        trgtbl = crticksbl + sideOrFront * ticksToMove;
//        trgtbr = crticksbr + ticksToMove;

        UpdateTicks();
        int trgtfl, trgtfr, trgtbl, trgtbr;
        if(x_mm != 0) //fata-spate
        {
            trgtfl = crticksfl + ticksToMove;
            trgtfr = crticksfr - ticksToMove;
            trgtbl = crticksbl - ticksToMove;
            trgtbr = crticksbr + ticksToMove;

        }
        else //stanga-dreapta
        {
            double offset = 0.6;
            trgtfl = (int)(crticksfl - ticksToMove*offset);
            trgtfr = (int)(crticksfr - ticksToMove*offset);
            trgtbl = crticksbl - ticksToMove;
            trgtbr = crticksbr - ticksToMove;
        }

        SwitchMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setTargetPosition(trgtfl);
        frontright.setTargetPosition(trgtfr);
        backleft.setTargetPosition(trgtbl);
        backright.setTargetPosition(trgtbr);
        SwitchMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        MotorValues mv = new MotorValues(motorPower);
        ApplyMotorValues(mv);

        while ((frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) && opModeIsActive()) {
            UpdateOrientation();
            UpdateTicks();
            telemetry.addData("fl", crticksfl +" "+ trgtfl);
            telemetry.addData("fr", crticksfr +" "+ trgtfr);
            telemetry.addData("bl", crticksbl +" "+ trgtbl);
            telemetry.addData("br", crticksbr +" "+ trgtbr);
            telemetry.update();
        }
        mv.applyPID(frontleft, crticksfl, trgtfl);
        mv.applyPID(frontright, crticksfr, trgtfr);
        mv.applyPID(backleft, crticksbl, trgtbl);
        mv.applyPID(backright, crticksbr, trgtbr);

        ApplyMotorValues(new MotorValues(0));
        UpdateTicks();
        UpdateOrientation();
    }

    public void AutonomousRotate(double angle) {

        ApplyMotorValues(new MotorValues(0));
        SwitchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double maxAngleError = 5;

        double crtAngle = gyro.getAngularOrientation().firstAngle;
        double sign = angle - crtAngle;
        double power = sign / Math.abs(sign) * 0.5;
        //MotorValues mv = new MotorValues(-power, -power, power, power, 0.5);
        //MotorValues mv = new MotorValues(power);

        //mv.NormaliseValues();
        //ApplyMotorValues(mv);
        while(Math.abs(crtAngle - angle) >= maxAngleError)
        {
            telemetry.addData("current angle", crtAngle);
            telemetry.addData("first second third angles", gyro.getAngularOrientation().firstAngle + " " + gyro.getAngularOrientation().secondAngle + " " + gyro.getAngularOrientation().thirdAngle);

            telemetry.addData("angle error", Math.abs(crtAngle - angle));
            telemetry.addData("sign", sign);
            telemetry.update();

            //mv.NormaliseValues();
            //ApplyMotorValues(mv);

            frontleft.setPower(-power);
            frontright.setPower(-power);
            backleft.setPower(power);
            backright.setPower(power);

            UpdateOrientation();
            UpdateTicks();
            crtAngle = gyro.getAngularOrientation().firstAngle;
        }
        ApplyMotorValues(new MotorValues(0));
        UpdateOrientation();
        UpdateTicks();
    }

    /*Functia care controleaza miscarea in TeleOp.
     * Citeste din gamepad1, nu are parametri*/
    public void TeleOpDrive() {
        SwitchMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        double power_fl = -x - y + rotation;
        double power_fr = -x + y + rotation;
        double power_bl = -x + y - rotation;
        double power_br = -x - y - rotation;

        MotorValues motorValues = new MotorValues(power_fl, power_fr, power_bl, power_br);
        if (gamepad1.left_bumper && rotation == 0) motorValues.SlowMode();

        motorValues.NormaliseValues();
        ApplyMotorValues(motorValues);
    }

    public void SwitchMotorModes(DcMotor.RunMode x) {
        frontleft.setMode(x);
        frontright.setMode(x);
        backleft.setMode(x);
        backright.setMode(x);

        while ((frontleft.getMode() != x || frontright.getMode() != x || backleft.getMode() != x || backright.getMode() != x) && opModeIsActive())
            ;
    }

    public double in_to_mm(double x) {
        return 25.4 * x;
    }

    public int mm_to_ticks(double x) {
        return (int) (((ticks_rev * x) / (diameter_mm * Math.PI)) * gear_ratio);
    }

    public void ApplyMotorValues(MotorValues motorValues) {
        frontleft.setPower(motorValues.fl);
        frontright.setPower(motorValues.fr);
        backleft.setPower(motorValues.bl);
        backright.setPower(motorValues.br);
    }

    public void UpdateTicks() {
        crticksfl = frontleft.getCurrentPosition();
        crticksfr = frontright.getCurrentPosition();
        crticksbl = backleft.getCurrentPosition();
        crticksbr = backright.getCurrentPosition();

        UpdateOrientation();

        /*telemetry.addData("Ticks Front Left:", crticksfl);
        telemetry.addData("Ticks Front Right:", crticksfr);
        telemetry.addData("Ticks Back Left:", crticksbl);
        telemetry.addData("Ticks Back Right:", crticksbr);
        telemetry.addData("Heading:", crtangle.thirdAngle);

        telemetry.update();*/
    }

    public void UpdateOrientation() {
        crtangle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void InitialiseVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        OpenCvPipeline pipeline = new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                return null;
            }
        };
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) { }
        });
    }

    public void StopVision() { webcam.stopStreaming(); }

//    public void addToTrafaletAngle(double angle)
//    {
//        trafaletAngle += angle;
//        trafaletServoStanga.setPosition(trafaletAngle/180);
//        trafaletServoDreapta.setPosition(trafaletAngle/180);
//    }

    public void addToRampaAngle(double angle)
    {
        rampaAngle += angle;
        rampaServoStanga.setPosition(rampaAngle/180);
        rampaServoDreapta.setPosition(gearRatioOffset*rampaAngle/180);
    }

    public void changeRampaState(int unit)
    {
        if(rampaState + unit >= 0 && rampaState + unit < 3) rampaState += unit;
        else return;

        addToRampaAngle(-rampaAngle + unghiuriRampa[rampaState]);
    }

    public void motorRampaOnOff(int direction)
    {
        mergeRampa = !mergeRampa;
        rampaMotorDreapta.setPower(mergeRampa ? direction : 0);
        rampaMotorStanga.setPower(mergeRampa ? direction : 0);
    }

    @Override
    public void runOpMode () throws InterruptedException {
    }
}