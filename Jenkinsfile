node {
    stage('Build') {
        if (env.BUILD_FLAVOR == "Release") {
          node('vic-os-release') {
            checkout scm
            sh './poky/build/build.sh build-victor-robot-factory-image' 
          }
        } else {
          node('vic-os-dev') {
            checkout scm
            sh './poky/build/build.sh'
          } 
        }
    }
    stage('Collect Artifacts') {
        archiveArtifacts artifacts: '_build/*.ota', onlyIfSuccessful: true, caseSensitive: true
    }
}