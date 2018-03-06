#The message sent from the YOLO image generator to the visual detection topic
std_msgs/Header header
# byte array of raw image
uint8[] image
string detected_class
# percentile rating of detection confidence
float32 confidence 
# coordinates of the top-left corner
int32 box_x_coord
int32 box_y_coord
int32 box_height
int32 box_width
