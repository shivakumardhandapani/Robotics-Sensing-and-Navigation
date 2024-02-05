// Generated by gencpp from file gps_driver/Customgps.msg
// DO NOT EDIT!


#ifndef GPS_DRIVER_MESSAGE_CUSTOMGPS_H
#define GPS_DRIVER_MESSAGE_CUSTOMGPS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace gps_driver
{
template <class ContainerAllocator>
struct Customgps_
{
  typedef Customgps_<ContainerAllocator> Type;

  Customgps_()
    : header()
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , utm_easting(0.0)
    , utm_northing(0.0)
    , zone(0)
    , letter()
    , hdop(0.0)
    , gpgga_read()  {
    }
  Customgps_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , utm_easting(0.0)
    , utm_northing(0.0)
    , zone(0)
    , letter(_alloc)
    , hdop(0.0)
    , gpgga_read(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _altitude_type;
  _altitude_type altitude;

   typedef double _utm_easting_type;
  _utm_easting_type utm_easting;

   typedef double _utm_northing_type;
  _utm_northing_type utm_northing;

   typedef uint8_t _zone_type;
  _zone_type zone;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _letter_type;
  _letter_type letter;

   typedef double _hdop_type;
  _hdop_type hdop;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _gpgga_read_type;
  _gpgga_read_type gpgga_read;





  typedef boost::shared_ptr< ::gps_driver::Customgps_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_driver::Customgps_<ContainerAllocator> const> ConstPtr;

}; // struct Customgps_

typedef ::gps_driver::Customgps_<std::allocator<void> > Customgps;

typedef boost::shared_ptr< ::gps_driver::Customgps > CustomgpsPtr;
typedef boost::shared_ptr< ::gps_driver::Customgps const> CustomgpsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gps_driver::Customgps_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gps_driver::Customgps_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gps_driver::Customgps_<ContainerAllocator1> & lhs, const ::gps_driver::Customgps_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.latitude == rhs.latitude &&
    lhs.longitude == rhs.longitude &&
    lhs.altitude == rhs.altitude &&
    lhs.utm_easting == rhs.utm_easting &&
    lhs.utm_northing == rhs.utm_northing &&
    lhs.zone == rhs.zone &&
    lhs.letter == rhs.letter &&
    lhs.hdop == rhs.hdop &&
    lhs.gpgga_read == rhs.gpgga_read;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gps_driver::Customgps_<ContainerAllocator1> & lhs, const ::gps_driver::Customgps_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gps_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gps_driver::Customgps_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gps_driver::Customgps_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_driver::Customgps_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_driver::Customgps_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_driver::Customgps_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_driver::Customgps_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gps_driver::Customgps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c13aa5d5b109c777f94aa4fa3948d681";
  }

  static const char* value(const ::gps_driver::Customgps_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc13aa5d5b109c777ULL;
  static const uint64_t static_value2 = 0xf94aa4fa3948d681ULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_driver::Customgps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gps_driver/Customgps";
  }

  static const char* value(const ::gps_driver::Customgps_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gps_driver::Customgps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"float64 latitude\n"
"float64 longitude\n"
"float64 altitude\n"
"float64 utm_easting\n"
"float64 utm_northing\n"
"uint8 zone\n"
"string letter\n"
"float64 hdop\n"
"string gpgga_read\n"
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

  static const char* value(const ::gps_driver::Customgps_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gps_driver::Customgps_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
      stream.next(m.utm_easting);
      stream.next(m.utm_northing);
      stream.next(m.zone);
      stream.next(m.letter);
      stream.next(m.hdop);
      stream.next(m.gpgga_read);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Customgps_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_driver::Customgps_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gps_driver::Customgps_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "utm_easting: ";
    Printer<double>::stream(s, indent + "  ", v.utm_easting);
    s << indent << "utm_northing: ";
    Printer<double>::stream(s, indent + "  ", v.utm_northing);
    s << indent << "zone: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.zone);
    s << indent << "letter: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.letter);
    s << indent << "hdop: ";
    Printer<double>::stream(s, indent + "  ", v.hdop);
    s << indent << "gpgga_read: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.gpgga_read);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GPS_DRIVER_MESSAGE_CUSTOMGPS_H