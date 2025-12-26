plugins {
    id("com.android.application")
    id("kotlin-android")
    // This line MUST have parentheses to link the Flutter SDK
    id("dev.flutter.flutter-gradle-plugin")
}

android {
    // This must match your package name exactly
    namespace = "com.petrobot.ai_pet_robot"
    compileSdk = 34

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_21
        targetCompatibility = JavaVersion.VERSION_21
    }

    kotlinOptions {
        jvmTarget = "21"
    }

    defaultConfig {
        applicationId = "com.petrobot.ai_pet_robot"
        minSdk = flutter.minSdkVersion 
        targetSdk = 34
        versionCode = 1
        versionName = "1.0.0"
    }
}

flutter {
    source = "../.."
}
