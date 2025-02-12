plugins {
    id("com.android.library")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "io.github.bionictigers"
    compileSdk = 34

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    // Configure test options
    testOptions {
        unitTests.isIncludeAndroidResources = true
    }

    defaultConfig {
        targetSdkVersion(rootProject.extra["defaultTargetSdkVersion"] as Int)
        minSdkVersion(rootProject.extra["defaultMinSdkVersion"] as Int)
    }
}

dependencies {
    // FTC dependencies remain unchanged
    implementation("org.firstinspires.ftc:RobotCore:10.1.1") {
        exclude(group = "org.nanohttpd", module = "nanohttpd")
    }
    implementation("org.firstinspires.ftc:RobotServer:10.1.1")
    implementation("org.firstinspires.ftc:OnBotJava:10.1.1")
    implementation("org.firstinspires.ftc:Hardware:10.1.1")
    implementation("org.firstinspires.ftc:FtcCommon:10.1.1")
    implementation("org.firstinspires.ftc:Vision:10.1.1")

    implementation("com.google.android.material:material:1.12.0")

    // Remove Jackson dependencies:
    // implementation("com.fasterxml.jackson.core:jackson-databind:2.18.2")
    // implementation("com.fasterxml.jackson.module:jackson-module-kotlin:2.18.2")

    // Add Moshi dependencies:
    implementation("com.squareup.moshi:moshi:1.14.0")
    implementation("com.squareup.moshi:moshi-kotlin:1.14.0")

    // Include the nanohttpd-websocket dependency but exclude its transitive nanohttpd dependency
    implementation("org.nanohttpd:nanohttpd-websocket:2.3.1") {
        exclude(group = "org.nanohttpd", module = "nanohttpd")
    }

    implementation(kotlin("reflect"))
}


// Helper functions if you use these dependency configurations
fun DependencyHandler.mockImplementation(dependencyNotation: Any) {
    add("mockImplementation", dependencyNotation)
}

fun DependencyHandler.prodImplementation(dependencyNotation: Any) {
    add("prodImplementation", dependencyNotation)
}
