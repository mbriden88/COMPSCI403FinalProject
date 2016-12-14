# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from final_project/GetCommandVelSrvRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg

class GetCommandVelSrvRequest(genpy.Message):
  _md5sum = "6abb3cb3c4d3a2838a634c9f26e13cdb"
  _type = "final_project/GetCommandVelSrvRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """sensor_msgs/Image Image
geometry_msgs/Point32 v0
geometry_msgs/Point32 w0

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z"""
  __slots__ = ['Image','v0','w0']
  _slot_types = ['sensor_msgs/Image','geometry_msgs/Point32','geometry_msgs/Point32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       Image,v0,w0

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetCommandVelSrvRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.Image is None:
        self.Image = sensor_msgs.msg.Image()
      if self.v0 is None:
        self.v0 = geometry_msgs.msg.Point32()
      if self.w0 is None:
        self.w0 = geometry_msgs.msg.Point32()
    else:
      self.Image = sensor_msgs.msg.Image()
      self.v0 = geometry_msgs.msg.Point32()
      self.w0 = geometry_msgs.msg.Point32()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.Image.header.seq, _x.Image.header.stamp.secs, _x.Image.header.stamp.nsecs))
      _x = self.Image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.Image.height, _x.Image.width))
      _x = self.Image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_BI().pack(_x.Image.is_bigendian, _x.Image.step))
      _x = self.Image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_6f().pack(_x.v0.x, _x.v0.y, _x.v0.z, _x.w0.x, _x.w0.y, _x.w0.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.Image is None:
        self.Image = sensor_msgs.msg.Image()
      if self.v0 is None:
        self.v0 = geometry_msgs.msg.Point32()
      if self.w0 is None:
        self.w0 = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.Image.header.seq, _x.Image.header.stamp.secs, _x.Image.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.Image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.Image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.Image.height, _x.Image.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.Image.encoding = str[start:end].decode('utf-8')
      else:
        self.Image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.Image.is_bigendian, _x.Image.step,) = _get_struct_BI().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.Image.data = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.v0.x, _x.v0.y, _x.v0.z, _x.w0.x, _x.w0.y, _x.w0.z,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.Image.header.seq, _x.Image.header.stamp.secs, _x.Image.header.stamp.nsecs))
      _x = self.Image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.Image.height, _x.Image.width))
      _x = self.Image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_BI().pack(_x.Image.is_bigendian, _x.Image.step))
      _x = self.Image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_6f().pack(_x.v0.x, _x.v0.y, _x.v0.z, _x.w0.x, _x.w0.y, _x.w0.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.Image is None:
        self.Image = sensor_msgs.msg.Image()
      if self.v0 is None:
        self.v0 = geometry_msgs.msg.Point32()
      if self.w0 is None:
        self.w0 = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.Image.header.seq, _x.Image.header.stamp.secs, _x.Image.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.Image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.Image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.Image.height, _x.Image.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.Image.encoding = str[start:end].decode('utf-8')
      else:
        self.Image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.Image.is_bigendian, _x.Image.step,) = _get_struct_BI().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.Image.data = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.v0.x, _x.v0.y, _x.v0.z, _x.w0.x, _x.w0.y, _x.w0.z,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f
_struct_BI = None
def _get_struct_BI():
    global _struct_BI
    if _struct_BI is None:
        _struct_BI = struct.Struct("<BI")
    return _struct_BI
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from final_project/GetCommandVelSrvResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class GetCommandVelSrvResponse(genpy.Message):
  _md5sum = "c587a607b40c5d69987ec528bee6ee8f"
  _type = "final_project/GetCommandVelSrvResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Point32 Cv
geometry_msgs/Point32 Cw


================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z"""
  __slots__ = ['Cv','Cw']
  _slot_types = ['geometry_msgs/Point32','geometry_msgs/Point32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       Cv,Cw

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetCommandVelSrvResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.Cv is None:
        self.Cv = geometry_msgs.msg.Point32()
      if self.Cw is None:
        self.Cw = geometry_msgs.msg.Point32()
    else:
      self.Cv = geometry_msgs.msg.Point32()
      self.Cw = geometry_msgs.msg.Point32()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6f().pack(_x.Cv.x, _x.Cv.y, _x.Cv.z, _x.Cw.x, _x.Cw.y, _x.Cw.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.Cv is None:
        self.Cv = geometry_msgs.msg.Point32()
      if self.Cw is None:
        self.Cw = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.Cv.x, _x.Cv.y, _x.Cv.z, _x.Cw.x, _x.Cw.y, _x.Cw.z,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6f().pack(_x.Cv.x, _x.Cv.y, _x.Cv.z, _x.Cw.x, _x.Cw.y, _x.Cw.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.Cv is None:
        self.Cv = geometry_msgs.msg.Point32()
      if self.Cw is None:
        self.Cw = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.Cv.x, _x.Cv.y, _x.Cv.z, _x.Cw.x, _x.Cw.y, _x.Cw.z,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f
class GetCommandVelSrv(object):
  _type          = 'final_project/GetCommandVelSrv'
  _md5sum = 'f3ec071038723cac47826a5f79b9a4b2'
  _request_class  = GetCommandVelSrvRequest
  _response_class = GetCommandVelSrvResponse