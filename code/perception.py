import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select_terrain = np.zeros_like(img[:,:,0])
    color_select_obstacle = np.copy(color_select_terrain)
    color_select_rock = np.copy(color_select_terrain)
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh_terrain = (img[:,:,0] > rgb_thresh[0]) \
                           & (img[:,:,1] > rgb_thresh[1]) \
                           & (img[:,:,2] > rgb_thresh[2])
    #
    above_thresh_obstacle = (img[:,:,0] < rgb_thresh[0]) \
                            & (img[:,:,1] < rgb_thresh[1]) \
                            & (img[:,:,2] < rgb_thresh[2])
    #
    above_thresh_rock = (img[:,:,0] > 140) \
                        & (img[:,:,2] < 60)
    # Index the array of zeros with the boolean array and set to 1
    color_select_terrain[above_thresh_terrain] = 1
    color_select_obstacle[above_thresh_obstacle] = 1
    color_select_rock[above_thresh_rock] = 1
    #print(color_select[])
    # Return the binary image
    return color_select_terrain, color_select_obstacle, color_select_rock

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    source = np.float32([[14 ,140 ], [ 300,140 ], [ 198,96 ], [ 118, 96]])
    destination = np.float32([[160 ,150 ], [ 170,150 ], [170 ,140 ], [ 160, 140]])
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    threshed_terrain, threshed_obstacle, threshed_rock = color_thresh(warped)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    threshold = 160
    Rover.vision_image[:,:,0] = threshed_obstacle - threshold
    Rover.vision_image[:,:,1] = threshed_rock - threshold
    Rover.vision_image[:,:,2] = threshed_terrain - threshold

    # print("xpix_terrain",xpix_terrain[1:5])

    # 5) Convert map image pixel values to rover-centric coords

    xpix_terrain, ypix_terrain = rover_coords(threshed_terrain)
    xpix_obstacle, ypix_obstacle = rover_coords(threshed_obstacle)
    xpix_rock, ypix_rock = rover_coords(threshed_rock)

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    x_terrain_world, y_terrain_world = pix_to_world(xpix_terrain,ypix_terrain,Rover.xpos[Rover.count],Rover.ypos[Rover.count],Rover.yaw[Rover.count],Rover.worldmap.shape[0], scale)
    x_obstacle_world, y_obstacle_world = pix_to_world(xpix_obstacle,ypix_obstacle,Rover.xpos[Rover.count],Rover.ypos[Rover.count],Rover.yaw[Rover.count],Rover.worldmap.shape[0], scale)
    x_rock_world, y_rock_world = pix_to_world(xpix_rock,ypix_rock,Rover.xpos[Rover.count],Rover.ypos[Rover.count],Rover.yaw[Rover.count],Rover.worldmap.shape[0], scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[y_terrain_world, x_terrain_world, 2] += 1
    Rover.worldmap[y_obstacle_world, x_obstacle_world, 0] += 1
    Rover.worldmap[y_rock_world, x_rock_world, 1] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles
    Rover.terrain_dists,Rover.terrain_angles = to_polar_coords (x_terrain_world, y_terrain_world)
    Rover.obstacle_dists,Rover.obstacle_angles = to_polar_coords (x_obstacle_world, y_obstacle_world)
    Rover.rock_dists,Rover.rock_angles = to_polar_coords (x_rock_world, y_rock_world)

    return Rover