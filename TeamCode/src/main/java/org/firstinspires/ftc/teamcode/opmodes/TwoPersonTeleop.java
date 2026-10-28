package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.IntakeController;
import org.firstinspires.ftc.teamcode.classes.LaunchController;
import org.firstinspires.ftc.teamcode.classes.MecanumDrive;

//@Disabled
@TeleOp(name = "Teleop 2 Person", group = "Linear OpMode")
public class TwoPersonTeleop extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();
    LaunchController launcher = new LaunchController();

    IntakeController intake = new IntakeController();

    private boolean fieldRelative = false;
    private boolean drive_mode_switched = false;

    @Override
    public void runOpMode(){
        drive.init(hardwareMap);
        launcher.init(hardwareMap);
        intake.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("\nWaiting For Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Movement
            if (fieldRelative){
                drive.driveFieldRelative(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
                if (gamepad1.y){
                    drive.resetYaw();
                }
            }
            else {
                drive.driveRobotRelative(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
            }

            // Swapping Control Mode
            if (gamepad1.b && (!drive_mode_switched)) {
                fieldRelative = !fieldRelative;
                drive_mode_switched = true;
            }
            else if (!gamepad1.b){
                drive_mode_switched = false;
            }

            if (gamepad2.b){
                launcher.stop_all();
            }

            intake.set_intake_speed(-gamepad2.left_stick_y);


            launcher.launch(gamepad2.y, -gamepad2.right_stick_y);

            telemetry.addData("Flywheel Speed: ", launcher.launcherVel());
            telemetry.addData("Servo Speed: ", launcher.servoSpeed());
            telemetry.addData("Launch State: ", launcher.getLaunchState());
            telemetry.addData("Intake Speed: ", intake.get_intake_speed());
            telemetry.update();


        }

    }

}
