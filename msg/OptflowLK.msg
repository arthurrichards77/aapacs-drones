Header header

# size of original image
int32 width
int32 height

# number of features in play
uint32 tags_count

# location of each feature (pixels)
int32[] tags_x
int32[] tags_y

# delta-pixel movement of each feature
int32[] tags_dx
int32[] tags_dy