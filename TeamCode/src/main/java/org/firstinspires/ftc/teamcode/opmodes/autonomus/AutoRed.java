package org.firstinspires.ftc.teamcode.opmodes.autonomus;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.Path;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.common.pedroPathing.Constants;


@Autonomous(name = "Red", group = "Autonomous")
public class AutoRed extends LinearOpMode{
    ///                                                                                                 ///
    /// FYI for you Tristen: Whey you are watching any sort of video on how to do Autonomous,           ///
    /// to only watch a video on Pedro Pathing 3.0, because the syntax for the 3.0 is totally different ///
    /// from the syntax for 2.0 or any other version ;) I would use this video:                         ///
    /// https://www.google.com/search?q=pedro+pathing+3.0+from+the+lumenaries&rlz=1C1VDKB_enUS1000US1000&oq=pedro+pathing+3.0+from+the+lumenaries&gs_lcrp=EgZjaHJvbWUyBggAEEUYOTIJCAEQIRgKGKABMgkIAhAhGAoYoAEyCQgDECEYChigATIGCAQQIRgK0gEIODMzOWowajeoAgCwAgA&sourceid=chrome&source=chrome.ob&ie=UTF-8#fpstate=ive&vld=cid:6278c5ea,vid:5kZFd3xaNjU,st:0         ///
    /// You've got this. God bless you on it :)                                                         ///
    ///                                                                                                 ///

    private Follower follower;

    /// sets the measurement of Pose heading to RADIANS (be sure to convert angles to radians!)
    private final PoseFactory poseFactory = PoseFactory.radians();

    /// use https://visualizer.pedropathing.com/ to access the visualizer to choose and mark out the path for Autonomous,
    /// as well ass to see the specific points (x,y,heading) of where you want the robot to be and to go

    private final Pose start = poseFactory.of(7.5344155844155765, 107.75592532467532, 0);
    private final Pose shoot = poseFactory.of(60.410389610389615, 108.03636363636363, 0);
    //private final Pose atFlower = new Pose(47.962987012987014, 127.2351461038961,Math.toRadians(4.712389));
    //private final Pose grabFromFlower = new Pose(x,y,Math.toRadians(heading));
    private final Pose returnToStart = start;


    private Path leave() {
        return line(start, shoot).linear(start, shoot);
    }

    private Path comeBack() {
        return line(shoot, returnToStart).linear(shoot, returnToStart);
    }
    /// Compound Path between leave() and comeBack()
    public Path leaveAndComeBack() {
        return Paths.path(leave(), comeBack());
    }

    /// These 2 voids are just to save some lines of code
    public void getPathReady() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
    }

    public void executePath() {
        follower.update();
        Scheduler.execute();
    }

    ///control pose for curves, not needed unless using curves (replace x,y,heading):
    //private final Pose controlPose1 = poseFactory.of(x,y,Math.toRadians(heading));

    /// Instructions for the robot to follow the simple "leave and return" autonomous path
    @Override
    public void runOpMode(){

        getPathReady();

        follower.setPose(start);
        schedule(follow(follower, leaveAndComeBack()));

        executePath();
    }
}