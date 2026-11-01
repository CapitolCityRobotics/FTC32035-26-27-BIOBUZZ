package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.IntakeController;
import org.firstinspires.ftc.teamcode.classes.LaunchController;
import org.firstinspires.ftc.teamcode.classes.MecanumDrive;

@Autonomous(name = "Auto2", group = "Auto")
@Disabled
public class Auto2 extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();
    LaunchController launcher = new LaunchController();
    IntakeController intake = new IntakeController();

    private String alliance = "Red";
    private String direction = "Up";

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);

        while (opModeInInit()) {
            telemetry.addLine("Press X for BLUE");
            telemetry.addLine("Press B for RED");
            telemetry.addData("Selected Alliance", alliance);
            telemetry.addLine("\nPress Y for UP");
            telemetry.addLine("Press A for DOWN");
            telemetry.addData("Selected location", direction);
            telemetry.update();

            if (gamepad1.x){
                alliance = "Blue";
            } else if (gamepad2.b){
                alliance = "Red";
            }
            if (gamepad1.y){
                direction = "Up";
            } else if (gamepad2.a){
                direction = "Down";
            }

            launcher.launch(true, 0, true);
        }

        }
}
