# line_finder
`line_finder` is a ros node that is supposed to run with the gazebo simulator (launch and urdf files can be found [here](https://github.com/KNR-Selfie/selfie_carolocup2019_simulator)) that detects the marked lanes of a road.

## Subscribed topics
`/image_raw`

## Raw image

Raw images come from a camera mounted on a car model in Gazebo
![im3](https://user-images.githubusercontent.com/17860903/55499304-faaec280-5645-11e9-9534-93a0d6fd71ce.png)

## Histogram with peak detection

The peaks are being marked with green vertical lines
![im1](https://user-images.githubusercontent.com/17860903/55499109-79efc680-5645-11e9-8a95-dc95c0bc7d1f.png)

## Detected lines

A second order polynomial is used to approximate the curve made by the center points of the rectangles 
![im2](https://user-images.githubusercontent.com/17860903/55499216-c4714300-5645-11e9-9abc-e13b7416b0c2.png)

