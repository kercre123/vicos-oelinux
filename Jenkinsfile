pipeline {
  agent any
  stages {
    stage('Get VM') {
      steps {
        node(label: 'vicos')
      }
    }
    stage('Build') {
      steps {
        sh './poky/build/build.sh'
        timeout(unit: 'MINUTES', time: 120)
      }
    }
    stage('Collect Artifacts') {
      steps {
        archiveArtifacts(artifacts: '_build/*.ota', onlyIfSuccessful: true, caseSensitive: true)
      }
    }
  }
}