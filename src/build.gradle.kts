plugins {
    kotlin("multiplatform") version "1.7.20"
}

repositories {
    mavenCentral()
}

kotlin {
    linuxX64("native") // on Linux
        binaries {
            executable()
        }
    }
}

dependencies {
    implementation("org.jetbrains.kotlinx:multik-core:0.2.0")
    implementation("org.jetbrains.kotlinx:multik-default:0.2.0")
}

tasks.withType<Wrapper> {
    gradleVersion = "6.7.1"
    distributionType = Wrapper.DistributionType.BIN
}
