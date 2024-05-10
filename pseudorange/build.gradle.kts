plugins {
    id("com.android.library")
}

android {
    namespace = "com.example.pseudorange"
    compileSdk = 33

    defaultConfig {
        minSdk = 24

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        consumerProguardFiles("consumer-rules.pro")
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
}

dependencies {

    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.12.0")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
    implementation(files("libs/protobuf-nano.jar"))
    implementation(files("libs/suplClient.jar"))

    implementation("com.google.guava:guava:31.1-jre") //To import com.google.common.base.Preconditions in GpsNavigationMessageStore class
    implementation("joda-time:joda-time:2.9.4")    // To import DateTime use in GpsTime class
}