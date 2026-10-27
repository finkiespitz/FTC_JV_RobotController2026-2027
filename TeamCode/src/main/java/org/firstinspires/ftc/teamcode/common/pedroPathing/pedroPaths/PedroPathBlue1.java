package org.firstinspires.ftc.teamcode.common.pedroPathing.pedroPaths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

public class PedroPathBlue1 {

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose gardenIntake = poseFactory.of(8, 8, -90);
    private final Pose gardenIntakeControl1 = poseFactory.of(3.1297, 39.6783, 0);
    private final Pose gardenIntakeControl2 = poseFactory.of(40.8954, 72.1897, 0);
    private final Pose shoot2 = poseFactory.of(60, 133, -90);
    private final Pose shoot2Control1 = poseFactory.of(33.6528, 47.5302, 0);
    private final Pose shoot2Control2 = poseFactory.of(1.4226, 126.9303, 0);
    private final Pose shoot2Control3 = poseFactory.of(60.0322, 106.5034, 0);
    private final Pose shoot2Segment1Heading = poseFactory.of(60, 133, -90);
    private final Pose shoot2Segment2Start = poseFactory.of(60, 133, -90);
    private final Pose shoot2Segment2End = poseFactory.of(60, 133, 0);
    private final Pose shoot2Segment3Start = poseFactory.of(60, 133, 0);
    private final Pose shoot2Segment3End = poseFactory.of(60, 133, -90);
    private final Pose parkStart = poseFactory.of(60, 133, -90);
    private final Pose park = poseFactory.of(15.7594, 119.9135, 0);
    private final Pose parkControl1 = poseFactory.of(62.6887, 104.738, 0);

    public Path gardenIntake() {
        return Paths.curve(start, gardenIntakeControl1, gardenIntakeControl2, gardenIntake).linear(start, gardenIntake);
    }

    public Path shoot2() {
        return Paths.curve(gardenIntake, shoot2Control1, shoot2Control2, shoot2Control3, shoot2).heading(Interpolator.piecewise().until(0.0507, Interpolator.constant(shoot2Segment1Heading)).until(0.3524, Interpolator.linear(shoot2Segment2Start, shoot2Segment2End)).until(1, Interpolator.linear(shoot2Segment3Start, shoot2Segment3End)));
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
