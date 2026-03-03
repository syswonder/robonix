// # This message holds a collection of N-dimensional points, which may
// # contain additional information such as normals, intensity, etc. The
// # point data is stored as a binary blob, its layout described by the
// # contents of the "fields" array.
// #
// # The point cloud data may be organized 2d (image-like) or 1d (unordered).
// # Point clouds organized as 2d images may be produced by camera depth sensors
// # such as stereo or time-of-flight.

// # Time of sensor data acquisition, and the coordinate frame ID (for 3d points).

#[derive(Debug, Serialize, Deserialize)]
pub struct PointCloud2 {
  pub header : super::std_msgs::Header, 

  // # 2D structure of the point cloud. If the cloud is unordered, height is
  // # 1 and width is the length of the point cloud.
  pub height : u32, 
  pub width : u32, 

  // # Describes the channels and their layout in the binary data blob.
  pub fields : Vec<PointField>, 

  pub is_bigendian : bool, // # Is this data bigendian?
  pub point_step : u32, // # Length of a point in bytes
  pub row_step : u32, // # Length of a row in bytes
  pub data : Vec<u8>, // # Actual point data, size is (row_step*height)

  pub is_dense : bool, // # True if there are no invalid points
}
