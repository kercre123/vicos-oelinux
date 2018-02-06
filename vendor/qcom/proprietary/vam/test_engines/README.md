Folder structure & file description
    .
    ├── test_engines
    │   ├── GroundTruthEngine
              Ground truth object detection, displays object name and location
    │   ├── TestEngine1
              Detects objects, counts objects, and generates an event
    │   ├── TestEngine2
              Counts and classifies objects, depends on object detection
    └── README
          This file.

# Choose Test Engine

_NOTE: Be sure to have performed all vam steps, if not already completed_

* See vam.

To choose test engine,

`$ adb shell`

`# cd /usr/lib64/VAMEngines`

`# ln -sf ../[libEngine_Sample] [libEngine_Sample].so`

e.g. # ln -sf ../libEngine_TestEngine1.so.0 libEngine_TestEngine1.so


To run the VA Sim test, see vam_sim.

