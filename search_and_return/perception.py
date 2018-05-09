import numpy as np
import cv2

#Constants
OBSTACLE_LAYER = 0
ROCK_LAYER = 1
DRIVABLE_LAYER = 2
LOWER_PITCH_ROLL = 0.5
UPPER_PITCH_ROLL = 359.5

kernel = np.ones((3 ,3), np.uint8)

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def drive_thresh(img, rgb_thresh=(155, 155, 155)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met

    kernel_sharpening = np.array([[-1, -1, -1],
                                  [-1, 9, -1],
                                  [-1, -1, -1]])

    sharpened = cv2.filter2D(img, -1, kernel_sharpening)

    color_select = np.zeros_like(sharpened[:, :, 0])


    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return cv2.erode(color_select, kernel, iterations=2)


def mapping_thresh(img, rgb_thresh=(190, 190, 190)):
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return cv2.erode(color_select, kernel, iterations=1)


def obstacle_thresh(img, rgb_thresh=(175, 175, 175)):
    color_select = np.zeros_like(img[:, :, 0])

    below_thresh = (img[:, :, 0] < rgb_thresh[0]) \
                   & (img[:, :, 1] < rgb_thresh[1]) \
                   & (img[:, :, 2] < rgb_thresh[2])

    color_select[below_thresh] = 1

    return cv2.erode(color_select, kernel, iterations=1)


def sample_thresh(img):
    kernel_sharpening = np.array([[-1, -1, -1],
                                  [-1, 9, -1],
                                  [-1, -1, -1]])

    #sharpened = cv2.filter2D(img, -1, kernel_sharpening)

    sharpened = img

    color_select = np.zeros_like(sharpened[:, :, 0])

    rgb_upper = (255, 255, 50)
    rgb_lower = (102, 102, 10)

    range_thresh = (sharpened[:, :, 0] < rgb_upper[0]) \
                   & (sharpened[:, :, 0] > rgb_lower[0]) \
                   & (sharpened[:, :, 1] < rgb_upper[1]) \
                   & (sharpened[:, :, 1] > rgb_lower[1]) \
                   & (sharpened[:, :, 2] < rgb_upper[2]) \
                   & (sharpened[:, :, 2] > rgb_lower[2])

    color_select[range_thresh] = 1

    #return cv2.morphologyEx(color_select, cv2.MORPH_CLOSE, kernel)
    return cv2.dilate(color_select, kernel, iterations=10)

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
    img = Rover.img

    # 1) Define source and destination points for perspective transform
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[img.shape[1] / 2 - dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1] / 2 + dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1] / 2 + dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
                              [img.shape[1] / 2 - dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
                              ])


    #threshold the rocks image before perspective
    pre_transform_rocks = sample_thresh(img)
    sample_rocks = perspect_transform(pre_transform_rocks, source, destination)

    # 2) Apply perspective transform
    perpective_img = perspect_transform(img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    road = drive_thresh(perpective_img)
    map = mapping_thresh(perpective_img)
    obstacles = obstacle_thresh(perpective_img)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:, :, OBSTACLE_LAYER] = obstacles
    Rover.vision_image[:, :, ROCK_LAYER] = sample_rocks
    Rover.vision_image[:, :, DRIVABLE_LAYER] = road

    # 5) Convert map image pixel values to rover-centric coords
    rover_obstacles_x, rover_obstacles_y = rover_coords(obstacles)
    rover_sample_rocks_x, rover_sample_rocks_y = rover_coords(sample_rocks)
    rover_drivable_x, rover_drivable_y = rover_coords(road)
    rover_map_x, rover_map_y = rover_coords(map)

    # 6) Convert rover-centric pixel values to world coordinates
    rover_xpos = Rover.pos[0]
    rover_ypos = Rover.pos[1]
    rover_yaw = Rover.yaw
    rover_roll = Rover.roll
    rover_pitch = Rover.pitch

    #tolerance for the rovers pitch and roll when updating the world map.  Updating the world map when these values are not near
    #zero can cause inaccurate data to be collected
    if (rover_roll <= LOWER_PITCH_ROLL or rover_roll >= UPPER_PITCH_ROLL) and (rover_pitch <= LOWER_PITCH_ROLL or rover_pitch >= UPPER_PITCH_ROLL):
        world_obstacles_x, world_obstacles_y = pix_to_world(rover_obstacles_x, rover_obstacles_y, rover_xpos, rover_ypos,
                                                            rover_yaw, Rover.worldmap.shape[0], 10)
        world_sample_rocks_x, world_sample_rocks_y = pix_to_world(rover_sample_rocks_x, rover_sample_rocks_y, rover_xpos,
                                                                  rover_ypos, rover_yaw, Rover.worldmap.shape[0], 10)

        world_map_x, world_map_y = pix_to_world(rover_map_x, rover_map_y, rover_xpos, rover_ypos, rover_yaw, Rover.worldmap.shape[0], 10)


        # 7) Update Rover worldmap (to be displayed on right side of screen)
        Rover.worldmap[world_obstacles_y, world_obstacles_x, OBSTACLE_LAYER] = 1
        Rover.worldmap[world_sample_rocks_y, world_sample_rocks_x, ROCK_LAYER] = 1
        Rover.worldmap[world_map_y, world_map_x, DRIVABLE_LAYER] = 1


    # 8) Convert rover-centric pixel positions to polar coordinates
    dist, angles = to_polar_coords(rover_drivable_x, rover_drivable_y)

    # Update Rover pixel distances and angles
    Rover.nav_dists = dist
    Rover.nav_angles = angles

    ob_dist, ob_angles = to_polar_coords(rover_obstacles_x, rover_obstacles_y)
    Rover.ob_angles = ob_angles
    Rover.ob_dists = ob_dist

    #do we perceive any samples?
    if sample_rocks.any():
        Rover.seeSample = True

        rock_dist, rock_angles = to_polar_coords(rover_sample_rocks_x, rover_sample_rocks_y)
        Rover.nav_dists = rock_dist
        Rover.nav_angles = rock_angles

    else:
        Rover.seeSample = False

    return Rover