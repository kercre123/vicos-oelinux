pipeline {
 agent {
    node {
        vSphereDeployFromTemplate {
            server('https://vsphere.ankicore.com')
            template('jenkins-vicos-slave')
            clone('clone')
            cluster('sjc-vm-cluster')
            datastore('sjc-san-build')
       }
       label 'vicos2'
    }
  }

  stages {
    stage('Build') {
      steps {
        sh './poky/build/build.sh'
        //timeout(unit: 'MINUTES', time: 60)
      }
    }
    stage('Collect Artifacts') {
      steps {
        archiveArtifacts(artifacts: '_build/*.ota', onlyIfSuccessful: true, caseSensitive: true)
      }
    }
  }

}
