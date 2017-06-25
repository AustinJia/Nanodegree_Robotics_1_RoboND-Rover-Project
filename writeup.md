## Project: Search and Sample Return



**The goals / steps of this project are the following: Navigate through the search map in autonomous mode, and keep the high fidelity rate and try to find as much as the rock, in total, there are 6 rocks.

**Training / Calibration:

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/youtube_video.jpg
[image5]: ./calibration_images/color_threshold_algo.jpg
[image4]: ./calibration_images/example_rock1.jpg
[image6]: ./calibration_images/helper_function_01.jpg


## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README


### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
Here is an example of how to include an image in your writeup.

![alt text][image1]

test data provided can be found in :misc/rover_image

I have several recorded image stack in: test_dataset. test_dataset_v1.test_dataset_v2.test_dataset_v3

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 


The following image shows the color threshold algorithm to identify terrain, obstacles and rock.
![alt text][image5]

The following is a short version of recorded video.
![Watch the video][YouTube Video](https://youtu.be/p5N4-tgY06c)

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  
* I typically running in resolution of 1024 * 768, and good graphic quality setting.

Steps :
* Have four different mode: forward, stop, stuck and picking.
* In forward mode, the rover will going forward.  Once rover don't have enough navigated terrain to go or have a mountains in front of the way, it will go to the stop mode.
* In stop mode, the rover will decrease the speed and yaw, after certain time if yaw doesnot work, it will turn left or right. once navigated terrain larger than certain value, it will go back to the forward mode.
* Rover will enter stuck mode. if rover's position doesn't move within 100 counter. in stuck mode, rover can turn left or right.
* Rover will enter stuck mode. if rover see rock or rock exist its surrounding area.

Helper function:
![alt text][image6]
* is_clear: check forward clear or not (above image).
* is_Stuck: check rover stuck or not using timer (stuck_counter).
* nearby_has_rock: check the surrounding area has rock or not.
* changeSamples_posToZero: check the position or -3, after the rock has found.

Future Work:
* Try to improve the fidelity, because I spend lots of time to improve the fidelity rate

