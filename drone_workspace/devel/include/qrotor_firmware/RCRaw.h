// Generated by gencpp from file qrotor_firmware/RCRaw.msg
// DO NOT EDIT!


#ifndef QROTOR_FIRMWARE_MESSAGE_RCRAW_H
#define QROTOR_FIRMWARE_MESSAGE_RCRAW_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace qrotor_firmware
{
template <class ContainerAllocator>
struct RCRaw_
{
  typedef RCRaw_<ContainerAllocator> Type;

  RCRaw_()
    : header()
    , values()  {
      values.assign(0);
  }
  RCRaw_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , values()  {
  (void)_alloc;
      values.assign(0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<uint16_t, 8>  _values_type;
  _values_type values;





  typedef boost::shared_ptr< ::qrotor_firmware::RCRaw_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qrotor_firmware::RCRaw_<ContainerAllocator> const> ConstPtr;

}; // struct RCRaw_

typedef ::qrotor_firmware::RCRaw_<std::allocator<void> > RCRaw;

typedef boost::shared_ptr< ::qrotor_firmware::RCRaw > RCRawPtr;
typedef boost::shared_ptr< ::qrotor_firmware::RCRaw const> RCRawConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qrotor_firmware::RCRaw_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qrotor_firmware::RCRaw_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::qrotor_firmware::RCRaw_<ContainerAllocator1> & lhs, const ::qrotor_firmware::RCRaw_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.values == rhs.values;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::qrotor_firmware::RCRaw_<ContainerAllocator1> & lhs, const ::qrotor_firmware::RCRaw_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace qrotor_firmware

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::qrotor_firmware::RCRaw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qrotor_firmware::RCRaw_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qrotor_firmware::RCRaw_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qrotor_firmware::RCRaw_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qrotor_firmware::RCRaw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qrotor_firmware::RCRaw_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qrotor_firmware::RCRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4e07e0a6c2de8828f77c94cd208f693e";
  }

  static const char* value(const ::qrotor_firmware::RCRaw_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4e07e0a6c2de8828ULL;
  static const uint64_t static_value2 = 0xf77c94cd208f693eULL;
};

template<class ContainerAllocator>
struct DataType< ::qrotor_firmware::RCRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qrotor_firmware/RCRaw";
  }

  static const char* value(const ::qrotor_firmware::RCRaw_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qrotor_firmware::RCRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# raw servo outputs\n"
"\n"
"Header header\n"
"uint16[8] values\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::qrotor_firmware::RCRaw_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qrotor_firmware::RCRaw_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.values);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RCRaw_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qrotor_firmware::RCRaw_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qrotor_firmware::RCRaw_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size(); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.values[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // QROTOR_FIRMWARE_MESSAGE_RCRAW_H