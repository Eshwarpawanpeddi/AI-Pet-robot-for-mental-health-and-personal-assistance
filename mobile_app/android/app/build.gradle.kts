plugins {
    id "com.android.application"
    id "kotlin-android"
    // This line links Flutter to your Android code and fixes the compilation errors
    id "dev.flutter.flutter-gradle-plugin"
}

android {
    namespace = "com.petrobot.ai_pet_robot"
    compileSdk = 34
    
    defaultConfig {
        applicationId = "com.petrobot.ai_pet_robot"
        minSdk = flutter.minSdkVersion
        targetSdk = 34
        versionCode = 1
        versionName = "1.0.0"
    }

    buildTypes {
        release {
            // Using debug keys for development release
            signingConfig = signingConfigs.debug
        }
    }
}

flutter {
    source = "../.."
}
