/* Auto-generated by genmsg_cpp for file /home/kuri/catkin_ws/src/owd_msgs/srv/SetFingerCompliant.srv */
#ifndef OWD_MSGS_SERVICE_SETFINGERCOMPLIANT_H
#define OWD_MSGS_SERVICE_SETFINGERCOMPLIANT_H
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
struct SetFingerCompliantRequest_ {
  typedef SetFingerCompliantRequest_<ContainerAllocator> Type;

  SetFingerCompliantRequest_()
  : enable(false)
  , strain(0)
  {
  }

  SetFingerCompliantRequest_(const ContainerAllocator& _alloc)
  : enable(false)
  , strain(0)
  {
  }

  typedef uint8_t _enable_type;
  uint8_t enable;

  typedef int32_t _strain_type;
  int32_t strain;


  typedef boost::shared_ptr< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetFingerCompliantRequest
typedef  ::owd_msgs::SetFingerCompliantRequest_<std::allocator<void> > SetFingerCompliantRequest;

typedef boost::shared_ptr< ::owd_msgs::SetFingerCompliantRequest> SetFingerCompliantRequestPtr;
typedef boost::shared_ptr< ::owd_msgs::SetFingerCompliantRequest const> SetFingerCompliantRequestConstPtr;



template <class ContainerAllocator>
struct SetFingerCompliantResponse_ {
  typedef SetFingerCompliantResponse_<ContainerAllocator> Type;

  SetFingerCompliantResponse_()
  : ok(false)
  , reason()
  {
  }

  SetFingerCompliantResponse_(const ContainerAllocator& _alloc)
  : ok(false)
  , reason(_alloc)
  {
  }

  typedef uint8_t _ok_type;
  uint8_t ok;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _reason_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  reason;


  typedef boost::shared_ptr< ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetFingerCompliantResponse
typedef  ::owd_msgs::SetFingerCompliantResponse_<std::allocator<void> > SetFingerCompliantResponse;

typedef boost::shared_ptr< ::owd_msgs::SetFingerCompliantResponse> SetFingerCompliantResponsePtr;
typedef boost::shared_ptr< ::owd_msgs::SetFingerCompliantResponse const> SetFingerCompliantResponseConstPtr;


struct SetFingerCompliant
{

typedef SetFingerCompliantRequest Request;
typedef SetFingerCompliantResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetFingerCompliant
} // namespace owd_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5a1540497b19861bc4577ccc4747bae7";
  }

  static const char* value(const  ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5a1540497b19861bULL;
  static const uint64_t static_value2 = 0xc4577ccc4747bae7ULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetFingerCompliantRequest";
  }

  static const char* value(const  ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool  enable\n\
int32 strain\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4679398f882e7cbdea165980d3ec2888";
  }

  static const char* value(const  ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4679398f882e7cbdULL;
  static const uint64_t static_value2 = 0xea165980d3ec2888ULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetFingerCompliantResponse";
  }

  static const char* value(const  ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ok\n\
string reason\n\
\n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.enable);
    stream.next(m.strain);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetFingerCompliantRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ok);
    stream.next(m.reason);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetFingerCompliantResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<owd_msgs::SetFingerCompliant> {
  static const char* value() 
  {
    return "5dcef267ed8d1dc073136b82b069876f";
  }

  static const char* value(const owd_msgs::SetFingerCompliant&) { return value(); } 
};

template<>
struct DataType<owd_msgs::SetFingerCompliant> {
  static const char* value() 
  {
    return "owd_msgs/SetFingerCompliant";
  }

  static const char* value(const owd_msgs::SetFingerCompliant&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5dcef267ed8d1dc073136b82b069876f";
  }

  static const char* value(const owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetFingerCompliant";
  }

  static const char* value(const owd_msgs::SetFingerCompliantRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5dcef267ed8d1dc073136b82b069876f";
  }

  static const char* value(const owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/SetFingerCompliant";
  }

  static const char* value(const owd_msgs::SetFingerCompliantResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // OWD_MSGS_SERVICE_SETFINGERCOMPLIANT_H

