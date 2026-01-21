package org.firstinspires.ftc.teamcode.OpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
public class StarterBotTeleopMecanums extends OpMode {

    final double INTAKE_FORWARD_VELOCITY = 0.8;
    final double INTAKE_BACKWARD_VELOCITY = -0.6;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = -1;
    boolean forward = true;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake;

    private CRServo pusher0, pusher1;

    //private Limelight3A limelight;
    private IMU imu;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    ElapsedTime feederTimer = new ElapsedTime();

    @Override
    public void init() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorfl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorfr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorbl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorbr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pusher0 = hardwareMap.get(CRServo.class, "pusher0");
        pusher1 = hardwareMap.get(CRServo.class, "pusher1");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

       // pusher0.setDirection(DcMotorSimple.Direction.REVERSE);
        pusher0.setPower(STOP_SPEED);
        pusher1.setPower(STOP_SPEED);

        telemetry.addData("Status", "Motors Initialized");
        telemetry.update();

       // limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);
        //limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "limelight"))
//                .setCameraResolution(new Size(640, 480))
//                .addProcessor(aprilTagProcessor)
//                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        mecanumDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        //LLResult result = limelight.getLatestResult();

//        if (result != null && result.isValid()) {
//            telemetry.addData("tx", result.getTx());
//            telemetry.addData("ty", result.getTy());
//
//            Pose3D botpose = result.getBotpose();
//            if (botpose != null) {
//                telemetry.addData("Botpose", botpose.toString());
//            }
//        }
//
//        // Python input/output SAFELY
//        if (result != null) {
//            double[] inputs = {1,2,3,4,5,6,7,8};
//            limelight.updatePythonInputs(inputs);
//
//            double[] pythonOutputs = result.getPythonOutput();
//            if (pythonOutputs != null && pythonOutputs.length > 0) {
//                telemetry.addData("Python Output", pythonOutputs[0]);
//            }
//        }
//
//        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        limelight.updateRobotOrientation(robotYaw);
//
//        if (result != null && result.isValid()) {
//            Pose3D mt2 = result.getBotpose_MT2();
//            if (mt2 != null) {
//                telemetry.addData("MT2 X", mt2.getPosition().x);
//                telemetry.addData("MT2 Y", mt2.getPosition().y);
//            }
//        }

        if (gamepad2.right_bumper) {
            forward = true;
            intake.setPower(0);
        }

        if (gamepad2.left_bumper) {
            if (forward) {
                forward = false;
                intake.setPower(INTAKE_FORWARD_VELOCITY);
            } else {
                forward = true;
                intake.setPower(INTAKE_BACKWARD_VELOCITY);
            }
        }
        if (gamepad2.x) {
            pusher0.setPower(FULL_SPEED);
            pusher1.setPower(FULL_SPEED);
        }
        else if (gamepad2.b){
            pusher0.setPower(STOP_SPEED);
            pusher1.setPower(STOP_SPEED);
        }

        telemetry.update();
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
