plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.20"
}

ihmc {
   group = "us.ihmc"
   version = "0.0.5"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.17.0")
   api("us.ihmc:ihmc-realtime:1.4.0")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-yovariables:0.9.11")
   api("us.ihmc:simulation-construction-set:0.21.9")
}

app.entrypoint("MicroStrain3DMVisualizer", "us.ihmc.visualizers.sensors.imu.lord.microstrain.MicroStrain3DMVisualizer")

tasks.register("runVisualizer", JavaExec::class.java) {
   classpath += ihmc.sourceSet("visualizers").runtimeClasspath
}