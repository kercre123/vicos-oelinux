node {
    stage('Build') {
        checkout scm
        sh './poky/build/build.sh'
    }
    stage('Collect Artifacts') {
        archiveArtifacts artifacts: '_build/*.ota', onlyIfSuccessful: true, caseSensitive: true
    }
}