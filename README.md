# line_finder
`line_finder` is a ros node that is supposed to run with the [gazebo simulator](https://github.com/KNR-Selfie/selfie_carolocup2019_simulator)  that detects the center lane
(in current version) and displays the whole process in three different windows. The first one is the unprocessed image, the second one is the same image but after the execution of the algorithm, and the third one is a histogram with marked peaks that show the location of the right and center lane.

## Subscribed topics
`/image_raw`
