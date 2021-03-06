// Generated by gencpp from file ds4_driver/Trackpad.msg
// DO NOT EDIT!


#ifndef DS4_DRIVER_MESSAGE_TRACKPAD_H
#define DS4_DRIVER_MESSAGE_TRACKPAD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ds4_driver
{
template <class ContainerAllocator>
struct Trackpad_
{
  typedef Trackpad_<ContainerAllocator> Type;

  Trackpad_()
    : id(0)
    , active(0)
    , x(0.0)
    , y(0.0)  {
    }
  Trackpad_(const ContainerAllocator& _alloc)
    : id(0)
    , active(0)
    , x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef uint16_t _id_type;
  _id_type id;

   typedef int32_t _active_type;
  _active_type active;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::ds4_driver::Trackpad_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ds4_driver::Trackpad_<ContainerAllocator> const> ConstPtr;

}; // struct Trackpad_

typedef ::ds4_driver::Trackpad_<std::allocator<void> > Trackpad;

typedef boost::shared_ptr< ::ds4_driver::Trackpad > TrackpadPtr;
typedef boost::shared_ptr< ::ds4_driver::Trackpad const> TrackpadConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ds4_driver::Trackpad_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ds4_driver::Trackpad_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ds4_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ds4_driver': ['/home/yoggi/hexapod_ws/src/ds4_driver/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ds4_driver::Trackpad_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ds4_driver::Trackpad_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ds4_driver::Trackpad_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ds4_driver::Trackpad_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ds4_driver::Trackpad_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ds4_driver::Trackpad_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ds4_driver::Trackpad_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7f8d46ab2334dfb3664bed321f9eaf05";
  }

  static const char* value(const ::ds4_driver::Trackpad_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7f8d46ab2334dfb3ULL;
  static const uint64_t static_value2 = 0x664bed321f9eaf05ULL;
};

template<class ContainerAllocator>
struct DataType< ::ds4_driver::Trackpad_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ds4_driver/Trackpad";
  }

  static const char* value(const ::ds4_driver::Trackpad_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ds4_driver::Trackpad_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Trackpad message for DualShock 4\n\
uint16 id       # Touch ID (increments every touch)\n\
int32 active    # 0: inactive, 1: active\n\
float32 x       # 0.0: left edge, 1.0: right edge\n\
float32 y       # 0.0: left edge, 1.0: right edge\n\
";
  }

  static const char* value(const ::ds4_driver::Trackpad_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ds4_driver::Trackpad_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.active);
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Trackpad_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ds4_driver::Trackpad_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ds4_driver::Trackpad_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.id);
    s << indent << "active: ";
    Printer<int32_t>::stream(s, indent + "  ", v.active);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DS4_DRIVER_MESSAGE_TRACKPAD_H
