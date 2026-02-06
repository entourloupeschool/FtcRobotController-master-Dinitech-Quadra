# ============================================================================
# ADDED 6 février 2026 - ProGuard rules optimized for FTC Control Hub
# ============================================================================

# Keep all FTC SDK classes
-keep class com.qualcomm.** { *; }
-keep class org.firstinspires.ftc.** { *; }

# Keep Hardware classes (motors, servos, sensors)
-keep class com.qualcomm.hardware.** { *; }
-keep class com.qualcomm.robotcore.hardware.** { *; }

# Keep OpMode annotations and classes
-keep @com.qualcomm.robotcore.eventloop.opmode.* class * { *; }
-keepclassmembers class * {
    @com.qualcomm.robotcore.eventloop.opmode.* *;
}

# Keep TeleOp and Autonomous annotations
-keep class * extends com.qualcomm.robotcore.eventloop.opmode.OpMode { *; }
-keep class * extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode { *; }

# Keep Vuforia and TensorFlow classes
-keep class org.firstinspires.ftc.robotcore.external.tfod.** { *; }
-keep class org.firstinspires.ftc.robotcore.external.navigation.** { *; }

# Keep Pedro Pathing library classes
-keep class com.pedropathing.** { *; }

# Keep FTCLib classes
-keep class com.arcrobotics.ftclib.** { *; }

# Keep custom TeamCode classes
-keep class org.firstinspires.ftc.teamcode.** { *; }

# Optimization settings for Control Hub
-optimizationpasses 5
-allowaccessmodification
-dontpreverify

# Remove logging in release builds to improve performance
-assumenosideeffects class android.util.Log {
    public static *** d(...);
    public static *** v(...);
    public static *** i(...);
}

# Keep line numbers for debugging stack traces
-keepattributes SourceFile,LineNumberTable

# ============================================================================
# END ADDED 6 février 2026
# ============================================================================
