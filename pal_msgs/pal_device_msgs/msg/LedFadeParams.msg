# RGBA of color, alpha will be used as intensity if supported by the led
std_msgs/ColorRGBA first_color
std_msgs/ColorRGBA second_color

#Duration of the transition from one color to the other
duration transition_duration

#Perform a fade when going from secondColor to firstColor
bool reverse_fade

