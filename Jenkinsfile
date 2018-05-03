pipeline {
 agent {
    node {
        vSphereDeployFromTemplate {
            server('https://vsphere.ankicore.com')
            template('js-vicos-IV')
            clone('clone')
            cluster('sjc-vm-04.ankicore.com')
            resourcepool('vicos-sjc-vm-04-test')
            datastore('jc-vm-04-localds')
            folder('vmos')
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
