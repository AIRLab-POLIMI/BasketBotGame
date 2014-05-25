/* Auto-generated by genmsg_cpp for file /home/matteo/catkin_ws/src/vrep/vrep_common/srv/simRosSetObjectParent.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSSETOBJECTPARENT_H
#define VREP_COMMON_SERVICE_SIMROSSETOBJECTPARENT_H
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
struct simRosSetObjectParentRequest_ {
  typedef simRosSetObjectParentRequest_<ContainerAllocator> Type;

  simRosSetObjectParentRequest_()
  : handle(0)
  , parentHandle(0)
  , keepInPlace(0)
  {
  }

  simRosSetObjectParentRequest_(const ContainerAllocator& _alloc)
  : handle(0)
  , parentHandle(0)
  , keepInPlace(0)
  {
  }

  typedef int32_t _handle_type;
  int32_t handle;

  typedef int32_t _parentHandle_type;
  int32_t parentHandle;

  typedef uint8_t _keepInPlace_type;
  uint8_t keepInPlace;


  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetObjectParentRequest
typedef  ::vrep_common::simRosSetObjectParentRequest_<std::allocator<void> > simRosSetObjectParentRequest;

typedef boost::shared_ptr< ::vrep_common::simRosSetObjectParentRequest> simRosSetObjectParentRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetObjectParentRequest const> simRosSetObjectParentRequestConstPtr;



template <class ContainerAllocator>
struct simRosSetObjectParentResponse_ {
  typedef simRosSetObjectParentResponse_<ContainerAllocator> Type;

  simRosSetObjectParentResponse_()
  : result(0)
  {
  }

  simRosSetObjectParentResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosSetObjectParentResponse
typedef  ::vrep_common::simRosSetObjectParentResponse_<std::allocator<void> > simRosSetObjectParentResponse;

typedef boost::shared_ptr< ::vrep_common::simRosSetObjectParentResponse> simRosSetObjectParentResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosSetObjectParentResponse const> simRosSetObjectParentResponseConstPtr;


struct simRosSetObjectParent
{

typedef simRosSetObjectParentRequest Request;
typedef simRosSetObjectParentResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosSetObjectParent
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "057df40a9de4d47ccec6ccbbf3e67baa";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x057df40a9de4d47cULL;
  static const uint64_t static_value2 = 0xcec6ccbbf3e67baaULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectParentRequest";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32 handle\n\
int32 parentHandle\n\
uint8 keepInPlace\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectParentResponse";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.handle);
    stream.next(m.parentHandle);
    stream.next(m.keepInPlace);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetObjectParentRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosSetObjectParentResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosSetObjectParent> {
  static const char* value() 
  {
    return "3fe38263068d68ac8ee5d5462c95bf28";
  }

  static const char* value(const vrep_common::simRosSetObjectParent&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosSetObjectParent> {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectParent";
  }

  static const char* value(const vrep_common::simRosSetObjectParent&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3fe38263068d68ac8ee5d5462c95bf28";
  }

  static const char* value(const vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectParent";
  }

  static const char* value(const vrep_common::simRosSetObjectParentRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3fe38263068d68ac8ee5d5462c95bf28";
  }

  static const char* value(const vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosSetObjectParent";
  }

  static const char* value(const vrep_common::simRosSetObjectParentResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSSETOBJECTPARENT_H

