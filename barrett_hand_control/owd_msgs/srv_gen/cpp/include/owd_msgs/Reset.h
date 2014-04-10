/* Auto-generated by genmsg_cpp for file /home/kuri/catkin_ws/src/owd_msgs/srv/Reset.srv */
#ifndef OWD_MSGS_SERVICE_RESET_H
#define OWD_MSGS_SERVICE_RESET_H
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
struct ResetRequest_ {
  typedef ResetRequest_<ContainerAllocator> Type;

  ResetRequest_()
  {
  }

  ResetRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::owd_msgs::ResetRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::ResetRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ResetRequest
typedef  ::owd_msgs::ResetRequest_<std::allocator<void> > ResetRequest;

typedef boost::shared_ptr< ::owd_msgs::ResetRequest> ResetRequestPtr;
typedef boost::shared_ptr< ::owd_msgs::ResetRequest const> ResetRequestConstPtr;



template <class ContainerAllocator>
struct ResetResponse_ {
  typedef ResetResponse_<ContainerAllocator> Type;

  ResetResponse_()
  : ok(false)
  , reason()
  {
  }

  ResetResponse_(const ContainerAllocator& _alloc)
  : ok(false)
  , reason(_alloc)
  {
  }

  typedef uint8_t _ok_type;
  uint8_t ok;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _reason_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  reason;


  typedef boost::shared_ptr< ::owd_msgs::ResetResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::ResetResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ResetResponse
typedef  ::owd_msgs::ResetResponse_<std::allocator<void> > ResetResponse;

typedef boost::shared_ptr< ::owd_msgs::ResetResponse> ResetResponsePtr;
typedef boost::shared_ptr< ::owd_msgs::ResetResponse const> ResetResponseConstPtr;


struct Reset
{

typedef ResetRequest Request;
typedef ResetResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Reset
} // namespace owd_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::ResetRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::ResetRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::owd_msgs::ResetRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/ResetRequest";
  }

  static const char* value(const  ::owd_msgs::ResetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::owd_msgs::ResetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::owd_msgs::ResetRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::ResetResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::ResetResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const  ::owd_msgs::ResetResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4679398f882e7cbdULL;
  static const uint64_t static_value2 = 0xea165980d3ec2888ULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/ResetResponse";
  }

  static const char* value(const  ::owd_msgs::ResetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ok\n\
string reason\n\
\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::ResetResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::ResetRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ResetRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::ResetResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ok);
    stream.next(m.reason);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ResetResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<owd_msgs::Reset> {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const owd_msgs::Reset&) { return value(); } 
};

template<>
struct DataType<owd_msgs::Reset> {
  static const char* value() 
  {
    return "owd_msgs/Reset";
  }

  static const char* value(const owd_msgs::Reset&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const owd_msgs::ResetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::ResetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/Reset";
  }

  static const char* value(const owd_msgs::ResetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const owd_msgs::ResetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::ResetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/Reset";
  }

  static const char* value(const owd_msgs::ResetResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // OWD_MSGS_SERVICE_RESET_H

