/* Auto-generated by genmsg_cpp for file /home/kuri/catkin_ws/src/owd_msgs/srv/SetSpeed.srv */
#ifndef OWD_MSGS_SERVICE_SETSPEED_H
#define OWD_MSGS_SERVICE_SETSPEED_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace owd_msgs
{
template <class ContainerAllocator>
struct SetSpeedRequest_ {
  typedef SetSpeedRequest_<ContainerAllocator> Type;

  SetSpeedRequest_()
  : velocities()
  , min_accel_time(0.0)
  {
  }

  SetSpeedRequest_(const ContainerAllocator& _alloc)
  : velocities(_alloc)
  , min_accel_time(0.0)
  {
  }

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _velocities_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  velocities;

  typedef double _min_accel_time_type;
  double min_accel_time;


  typedef boost::shared_ptr< ::owd_msgs::SetSpeedRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::SetSpeedRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetSpeedRequest
typedef  ::owd_msgs::SetSpeedRequest_<std::allocator<void> > SetSpeedRequest;

typedef boost::shared_ptr< ::owd_msgs::SetSpeedRequest> SetSpeedRequestPtr;
typedef boost::shared_ptr< ::owd_msgs::SetSpeedRequest const> SetSpeedRequestConstPtr;



template <class ContainerAllocator>
struct SetSpeedResponse_ {
  typedef SetSpeedResponse_<ContainerAllocator> Type;

  SetSpeedResponse_()
  : ok(false)
  , reason()
  {
  }

  SetSpeedResponse_(const ContainerAllocator& _alloc)
  : ok(false)
  , reason(_alloc)
  {
  }

  typedef uint8_t _ok_type;
  uint8_t ok;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _reason_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  reason;


  typedef boost::shared_ptr< ::owd_msgs::SetSpeedResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::SetSpeedResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetSpeedResponse
typedef  ::owd_msgs::SetSpeedResponse_<std::allocator<void> > SetSpeedResponse;

typedef boost::shared_ptr< ::owd_msgs::SetSpeedResponse> SetSpeedResponsePtr;
typedef boost::shared_ptr< ::owd_msgs::SetSpeedResponse const> SetSpeedResponseConstPtr;


struct SetSpeed
{

typedef SetSpeedRequest Request;
typedef SetSpeedResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetSpeed
} // namespace owd_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetSpeedRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetSpeedRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::SetSpeedRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f25539e8e9259120cd8f44c968d7cd7a";
  }

  static const char* value(const  ::owd_msgs::SetSpeedRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf25539e8e9259120ULL;
  static const uint64_t static_value2 = 0xcd8f44c968d7cd7aULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::SetSpeedRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetSpeedRequest";
  }

  static const char* value(const  ::owd_msgs::SetSpeedRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::SetSpeedRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64[] velocities\n\
float64 min_accel_time\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::SetSpeedRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetSpeedResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetSpeedResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::SetSpeedResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const  ::owd_msgs::SetSpeedResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4679398f882e7cbdULL;
  static const uint64_t static_value2 = 0xea165980d3ec2888ULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::SetSpeedResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetSpeedResponse";
  }

  static const char* value(const  ::owd_msgs::SetSpeedResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::SetSpeedResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ok\n\
string reason\n\
\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::SetSpeedResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::SetSpeedRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.velocities);
    stream.next(m.min_accel_time);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetSpeedRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::SetSpeedResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ok);
    stream.next(m.reason);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetSpeedResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<owd_msgs::SetSpeed> {
  static const char* value() 
  {
    return "98432a87a24b80d916ec91158921504f";
  }

  static const char* value(const owd_msgs::SetSpeed&) { return value(); } 
};

template<>
struct DataType<owd_msgs::SetSpeed> {
  static const char* value() 
  {
    return "owd_msgs/SetSpeed";
  }

  static const char* value(const owd_msgs::SetSpeed&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::SetSpeedRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "98432a87a24b80d916ec91158921504f";
  }

  static const char* value(const owd_msgs::SetSpeedRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::SetSpeedRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetSpeed";
  }

  static const char* value(const owd_msgs::SetSpeedRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::SetSpeedResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "98432a87a24b80d916ec91158921504f";
  }

  static const char* value(const owd_msgs::SetSpeedResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::SetSpeedResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetSpeed";
  }

  static const char* value(const owd_msgs::SetSpeedResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // OWD_MSGS_SERVICE_SETSPEED_H

