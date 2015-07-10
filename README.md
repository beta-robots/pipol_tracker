pipol_tracker
=============

Multi target people tracker for mobile robots. It uses multiple detector modalities, it is based on particle filtering and outputs a set of nearby people positions.

Although the inputs are configurable, currently it uses:

* Laser-based leg detections
* Monocular vision body detections (bounding box, bearing)
* Monocular vision face detections (3D thanks to eye distance detection)
* Depth camera 3D body detections
* Odometry

For further instructions (installation, etc) check the [pipol_tracker wiki] (https://github.com/beta-robots/pipol_tracker/wiki).
