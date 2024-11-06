package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;

//extends standard setup op mode
public class AutonomousOpMode extends StandardSetupOpMode{

    public enum COLOR {
        RED,
        BLUE
    }

    public enum SIDE {
        LEFT,
        RIGHT
    }

    protected COLOR color;
    protected SIDE side;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Every autonomous will drive forward, lift the shoulder, and
        // extend the arms at the same time.  Once the arms are
        // extended the shoulder is dropped slightly until the sample
        // is latched onto the sample bar.  Then the arms are retracted
        // while the fingers release the sample.
        ParallelAction sampleDrop = new ParallelAction(
                legs.actionBuilder(legs.pose)
                        .lineToX(20)
                        .build(),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        shoulder.setMode(Shoulder.Mode.HIGH_BAR);
                        return false;
                    }
                },
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        arm.setPosition(0.9,1000);
                        sleep(500);
                        shoulder.dropSample();
                        sleep(100);
                        hand.release(500);
                        return false;
                    }
                }
        );
        Actions.runBlocking(sampleDrop);

        // This action will grab a sample from the submersible
        // and then stow for travel (retract the arm and set the shoulder)
        ParallelAction grabSample = new ParallelAction(
                // Makayla said move forward until we are touching the submersible
                legs.actionBuilder(legs.pose)
                        .lineToX(25)
                        .build(),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        shoulder.setMode(Shoulder.Mode.SEARCH);
                        //hand.search();
                        //hand.pickup();
                        arm.gotoMin(0.5);
                        return false;
                    }
                }
        );
        Actions.runBlocking(grabSample);
    }

    /**
     * Method sets up this specific autonomous class
     * @param color color robot should use (and always yellow)
     * @param side side robot is starting on
     */
   protected void setup(COLOR color, SIDE side) {
       // Always ignore gamepads for autonomous
       this.ignoreGamepad = true;

       // Setup side and color for this autonomous opmode
       this.color = color;
       this.side = side;
   }
}
