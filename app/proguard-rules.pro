# Keep Ktor and kotlinx serialization reflective metadata where needed
-keep class kotlinx.serialization.** { *; }
-keep class io.ktor.** { *; }
-dontwarn org.slf4j.**
-dontwarn kotlin.**

