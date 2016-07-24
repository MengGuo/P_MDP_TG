#!/bin/sh

ffmpeg -r 0.7 -f image2 -i frame%d.png -s 1000x1000 -y simulation.avi
