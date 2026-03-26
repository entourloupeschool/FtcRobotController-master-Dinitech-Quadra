package org.firstinspires.ftc.teamcode.dinitech.other;

public class FieldDefinitions {
    public static final double TILE_DIM = 24;
    public static final double FIELD_SIDE_LENGTH = 144.0;

    /**
     * A launch zone is a triangular surface from which the robot is allowed to shoot. It is defined by 3 vectors. Inside the triangle, the robot can shoot.
     */
    public static final Globals.Vec2[] LAUNCH_ZONE_BIG = new Globals.Vec2[] {
            new Globals.Vec2(10, 133),
            new Globals.Vec2(72, 72),
            new Globals.Vec2(133, 133)
    };
    public static final Globals.Vec2[] LAUNCH_ZONE_SMALL = new Globals.Vec2[] {
            new Globals.Vec2(59, 0),
            new Globals.Vec2(72, 23),
            new Globals.Vec2(85, 0)
    };
}
