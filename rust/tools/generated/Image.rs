// # This message contains an uncompressed image
// # (0, 0) is at top-left corner of image


#[derive(Debug, Serialize, Deserialize)]
pub struct Image {
  pub header : super::std_msgs::Header, // # Header timestamp should be acquisition time of image
  // # Header frame_id should be optical frame of camera
  // # origin of frame should be optical center of cameara
  // # +x should point to the right in the image
  // # +y should point down in the image
  // # +z should point into to plane of the image
  // # If the frame_id here and the frame_id of the CameraInfo
  // # message associated with the image conflict
  // # the behavior is undefined

  pub height : u32, // # image height, that is, number of rows
  pub width : u32, // # image width, that is, number of columns

  // # The legal values for encoding are in file src/image_encodings.cpp
  // # If you want to standardize a new string format, join
  // # ros-users@lists.ros.org and send an email proposing a new encoding.

  pub encoding : String, // # Encoding of pixels -- channel meaning, ordering, size
  // # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

  pub is_bigendian : u8, // # is this data bigendian?
  pub step : u32, // # Full row length in bytes
  pub data : Vec<u8>, // # actual matrix data, size is (step * rows)
}
