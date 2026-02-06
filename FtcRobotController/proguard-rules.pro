# ============================================================================
# ADDED 6 février 2026 - ProGuard rules for FtcRobotController module
# ============================================================================

# Keep all FTC SDK core classes
-keep class com.qualcomm.** { *; }
-keep class org.firstinspires.ftc.** { *; }

# Keep all hardware interfaces
-keep class com.qualcomm.robotcore.hardware.** { *; }

# Keep all event loop and OpMode infrastructure
-keep class com.qualcomm.robotcore.eventloop.** { *; }

# Keep annotations
-keepattributes *Annotation*

# Keep line numbers for debugging
-keepattributes SourceFile,LineNumberTable

# ============================================================================
# END ADDED 6 février 2026
# ============================================================================
