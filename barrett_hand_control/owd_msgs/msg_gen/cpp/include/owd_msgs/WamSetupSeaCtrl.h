/* Auto-generated by genmsg_cpp for file /home/kuri/catkin_ws/src/owd_msgs/msg/WamSetupSeaCtrl.msg */
#ifndef OWD_MSGS_MESSAGE_WAMSETUPSEACTRL_H
#define OWD_MSGS_MESSAGE_WAMSETUPSEACTRL_H
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


namespace owd_msgs
{
template <class ContainerAllocator>
struct WamSetupSeaCtrl_ {
  typedef WamSetupSeaCtrl_<ContainerAllocator> Type;

  WamSetupSeaCtrl_()
  : jointIndices()
  , values()
  {
  }

  WamSetupSeaCtrl_(const ContainerAllocator& _alloc)
  : jointIndices(_alloc)
  , values(_alloc)
  {
  }

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _jointIndices_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  jointIndices;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _values_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  values;

  enum { retainArmState = 0 };

  typedef boost::shared_ptr< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct WamSetupSeaCtrl
typedef  ::owd_msgs::WamSetupSeaCtrl_<std::allocator<void> > WamSetupSeaCtrl;

typedef boost::shared_ptr< ::owd_msgs::WamSetupSeaCtrl> WamSetupSeaCtrlPtr;
typedef boost::shared_ptr< ::owd_msgs::WamSetupSeaCtrl const> WamSetupSeaCtrlConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace owd_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6714f77d58bb6129af5ebca9f942b31e";
  }

  static const char* value(const  ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6714f77d58bb6129ULL;
  static const uint64_t static_value2 = 0xaf5ebca9f942b31eULL;
};

template<class ContainerAllocator>
struct DataType< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "owd_msgs/WamSetupSeaCtrl";
  }

  static const char* value(const  ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 retainArmState = 0\n\
int32[] jointIndices\n\
float64[] values \n\
\n\
";
  }

  static const char* value(const  ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.jointIndices);
    stream.next(m.values);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct WamSetupSeaCtrl_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::owd_msgs::WamSetupSeaCtrl_<ContainerAllocator> & v) 
  {
    s << indent << "jointIndices[]" << std::endl;
    for (size_t i = 0; i < v.jointIndices.size(); ++i)
    {
      s << indent << "  jointIndices[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.jointIndices[i]);
    }
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size(); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.values[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // OWD_MSGS_MESSAGE_WAMSETUPSEACTRL_H

