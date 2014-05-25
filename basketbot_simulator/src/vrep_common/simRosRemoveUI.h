/* Auto-generated by genmsg_cpp for file /home/matteo/catkin_ws/src/vrep/vrep_common/srv/simRosRemoveUI.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSREMOVEUI_H
#define VREP_COMMON_SERVICE_SIMROSREMOVEUI_H
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




namespace vrep_common
{
template <class ContainerAllocator>
struct simRosRemoveUIRequest_ {
  typedef simRosRemoveUIRequest_<ContainerAllocator> Type;

  simRosRemoveUIRequest_()
  : handle(0)
  {
  }

  simRosRemoveUIRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;


  typedef boost::shared_ptr< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosRemoveUIRequest
typedef  ::vrep_common::simRosRemoveUIRequest_<std::allocator<void> > simRosRemoveUIRequest;

typedef boost::shared_ptr< ::vrep_common::simRosRemoveUIRequest> simRosRemoveUIRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosRemoveUIRequest const> simRosRemoveUIRequestConstPtr;



template <class ContainerAllocator>
struct simRosRemoveUIResponse_ {
  typedef simRosRemoveUIResponse_<ContainerAllocator> Type;

  simRosRemoveUIResponse_()
  : result(0)
  {
  }

  simRosRemoveUIResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosRemoveUIResponse
typedef  ::vrep_common::simRosRemoveUIResponse_<std::allocator<void> > simRosRemoveUIResponse;

typedef boost::shared_ptr< ::vrep_common::simRosRemoveUIResponse> simRosRemoveUIResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosRemoveUIResponse const> simRosRemoveUIResponseConstPtr;


struct simRosRemoveUI
{

typedef simRosRemoveUIRequest Request;
typedef simRosRemoveUIResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosRemoveUI
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "92535b678299d2bdda959704e78c275e";
  }

  static const char* value(const  ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x92535b678299d2bdULL;
  static const uint64_t static_value2 = 0xda959704e78c275eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosRemoveUIRequest";
  }

  static const char* value(const  ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 handle\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosRemoveUIResponse";
  }

  static const char* value(const  ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosRemoveUIRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosRemoveUIRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosRemoveUIResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosRemoveUIResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosRemoveUI> {
  static const char* value() 
  {
    return "6833353cd429b83819dab869600ce745";
  }

  static const char* value(const vrep_common::simRosRemoveUI&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosRemoveUI> {
  static const char* value() 
  {
    return "vrep_common/simRosRemoveUI";
  }

  static const char* value(const vrep_common::simRosRemoveUI&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosRemoveUIRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6833353cd429b83819dab869600ce745";
  }

  static const char* value(const vrep_common::simRosRemoveUIRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosRemoveUIRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosRemoveUI";
  }

  static const char* value(const vrep_common::simRosRemoveUIRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosRemoveUIResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6833353cd429b83819dab869600ce745";
  }

  static const char* value(const vrep_common::simRosRemoveUIResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosRemoveUIResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosRemoveUI";
  }

  static const char* value(const vrep_common::simRosRemoveUIResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSREMOVEUI_H

