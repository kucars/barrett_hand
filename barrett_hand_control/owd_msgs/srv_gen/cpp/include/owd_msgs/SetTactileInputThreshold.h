/* Auto-generated by genmsg_cpp for file /home/kuri/catkin_ws/src/owd_msgs/srv/SetTactileInputThreshold.srv */
#ifndef OWD_MSGS_SERVICE_SETTACTILEINPUTTHRESHOLD_H
#define OWD_MSGS_SERVICE_SETTACTILEINPUTTHRESHOLD_H
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
struct SetTactileInputThresholdRequest_ {
  typedef SetTactileInputThresholdRequest_<ContainerAllocator> Type;

  SetTactileInputThresholdRequest_()
  : pad_number(0)
  , threshold(0.0)
  , minimum_readings(0)
  {
  }

  SetTactileInputThresholdRequest_(const ContainerAllocator& _alloc)
  : pad_number(0)
  , threshold(0.0)
  , minimum_readings(0)
  {
  }

  typedef int32_t _pad_number_type;
  int32_t pad_number;

  typedef float _threshold_type;
  float threshold;

  typedef int32_t _minimum_readings_type;
  int32_t minimum_readings;


  typedef boost::shared_ptr< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetTactileInputThresholdRequest
typedef  ::owd_msgs::SetTactileInputThresholdRequest_<std::allocator<void> > SetTactileInputThresholdRequest;

typedef boost::shared_ptr< ::owd_msgs::SetTactileInputThresholdRequest> SetTactileInputThresholdRequestPtr;
typedef boost::shared_ptr< ::owd_msgs::SetTactileInputThresholdRequest const> SetTactileInputThresholdRequestConstPtr;



template <class ContainerAllocator>
struct SetTactileInputThresholdResponse_ {
  typedef SetTactileInputThresholdResponse_<ContainerAllocator> Type;

  SetTactileInputThresholdResponse_()
  : ok(false)
  , reason()
  {
  }

  SetTactileInputThresholdResponse_(const ContainerAllocator& _alloc)
  : ok(false)
  , reason(_alloc)
  {
  }

  typedef uint8_t _ok_type;
  uint8_t ok;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _reason_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  reason;


  typedef boost::shared_ptr< ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetTactileInputThresholdResponse
typedef  ::owd_msgs::SetTactileInputThresholdResponse_<std::allocator<void> > SetTactileInputThresholdResponse;

typedef boost::shared_ptr< ::owd_msgs::SetTactileInputThresholdResponse> SetTactileInputThresholdResponsePtr;
typedef boost::shared_ptr< ::owd_msgs::SetTactileInputThresholdResponse const> SetTactileInputThresholdResponseConstPtr;


struct SetTactileInputThreshold
{

typedef SetTactileInputThresholdRequest Request;
typedef SetTactileInputThresholdResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetTactileInputThreshold
} // namespace owd_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "12ef4e765b52e2818bb6194890021669";
  }

  static const char* value(const  ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x12ef4e765b52e281ULL;
  static const uint64_t static_value2 = 0x8bb6194890021669ULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetTactileInputThresholdRequest";
  }

  static const char* value(const  ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 pad_number\n\
float32 threshold\n\
int32 minimum_readings\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const  ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4679398f882e7cbdULL;
  static const uint64_t static_value2 = 0xea165980d3ec2888ULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetTactileInputThresholdResponse";
  }

  static const char* value(const  ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ok\n\
string reason\n\
\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.pad_number);
    stream.next(m.threshold);
    stream.next(m.minimum_readings);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetTactileInputThresholdRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ok);
    stream.next(m.reason);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetTactileInputThresholdResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<owd_msgs::SetTactileInputThreshold> {
  static const char* value() 
  {
    return "7205417324d66b6ed0188e5a03a48908";
  }

  static const char* value(const owd_msgs::SetTactileInputThreshold&) { return value(); } 
};

template<>
struct DataType<owd_msgs::SetTactileInputThreshold> {
  static const char* value() 
  {
    return "owd_msgs/SetTactileInputThreshold";
  }

  static const char* value(const owd_msgs::SetTactileInputThreshold&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7205417324d66b6ed0188e5a03a48908";
  }

  static const char* value(const owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetTactileInputThreshold";
  }

  static const char* value(const owd_msgs::SetTactileInputThresholdRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7205417324d66b6ed0188e5a03a48908";
  }

  static const char* value(const owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetTactileInputThreshold";
  }

  static const char* value(const owd_msgs::SetTactileInputThresholdResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // OWD_MSGS_SERVICE_SETTACTILEINPUTTHRESHOLD_H

