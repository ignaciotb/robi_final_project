# Contains information about the class of a found object, along with its confidence and ROI of detection

# class: The respective class type of the found object
string object_class

# confidence: how sure you are it is that object and not another one.
# It is between 0 and 1 and the closer to one it is the better
float32 confidence

# bounding_box: The region of the image, where the object is found
sensor_msgs/RegionOfInterest bounding_box
