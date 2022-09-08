plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
}

ihmc {
   group = "us.ihmc"
   version = "17-0.0.7"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.19.0")
   api("us.ihmc:ihmc-realtime:1.5.0")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:simulation-construction-set:0.22.8")
}

app.entrypoint("MicroStrain3DMVisualizer", "us.ihmc.visualizers.sensors.imu.lord.microstrain.MicroStrain3DMVisualizer")

tasks.register("runVisualizer", JavaExec::class.java) {
   classpath += ihmc.sourceSet("visualizers").runtimeClasspath
}
