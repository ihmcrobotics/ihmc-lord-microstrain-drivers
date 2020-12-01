plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.17"
}

ihmc {
   group = "us.ihmc"
   version = "0.0.2"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:ihmc-realtime:1.3.0")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-yovariables:0.9.6")
   api("us.ihmc:simulation-construction-set:0.21.1")
}

app.entrypoint("MicroStrain3DMVisualizer", "us.ihmc.visualizers.sensors.imu.lord.microstrain.MicroStrain3DMVisualizer")

tasks.register("run", JavaExec::class.java) {
   classpath += ihmc.sourceSet("visualizers").runtimeClasspath
}