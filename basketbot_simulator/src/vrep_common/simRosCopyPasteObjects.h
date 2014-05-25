/* Auto-generated by genmsg_cpp for file /home/matteo/catkin_ws/src/vrep/vrep_common/srv/simRosCopyPasteObjects.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSCOPYPASTEOBJECTS_H
#define VREP_COMMON_SERVICE_SIMROSCOPYPASTEOBJECTS_H
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
struct simRosCopyPasteObjectsRequest_ {
  typedef simRosCopyPasteObjectsRequest_<ContainerAllocator> Type;

  simRosCopyPasteObjectsRequest_()
  : objectHandles()
  {
  }

  simRosCopyPasteObjectsRequest_(const ContainerAllocator& _alloc)
  : objectHandles(_alloc)
  {
  }

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _objectHandles_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  objectHandles;


  typedef boost::shared_ptr< ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosCopyPasteObjectsRequest
typedef  ::vrep_common::simRosCopyPasteObjectsRequest_<std::allocator<void> > simRosCopyPasteObjectsRequest;

typedef boost::shared_ptr< ::vrep_common::simRosCopyPasteObjectsRequest> simRosCopyPasteObjectsRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosCopyPasteObjectsRequest const> simRosCopyPasteObjectsRequestConstPtr;



template <class ContainerAllocator>
struct simRosCopyPasteObjectsResponse_ {
  typedef simRosCopyPasteObjectsResponse_<ContainerAllocator> Type;

  simRosCopyPasteObjectsResponse_()
  : result(0)
  , newObjectHandles()
  {
  }

  simRosCopyPasteObjectsResponse_(const ContainerAllocator& _alloc)
  : result(0)
  , newObjectHandles(_alloc)
  {
  }

  typedef int32_t _result_type;
  int32_t result;

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _newObjectHandles_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  newObjectHandles;


  typedef boost::shared_ptr< ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosCopyPasteObjectsResponse
typedef  ::vrep_common::simRosCopyPasteObjectsResponse_<std::allocator<void> > simRosCopyPasteObjectsResponse;

typedef boost::shared_ptr< ::vrep_common::simRosCopyPasteObjectsResponse> simRosCopyPasteObjectsResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosCopyPasteObjectsResponse const> simRosCopyPasteObjectsResponseConstPtr;


struct simRosCopyPasteObjects
{

typedef simRosCopyPasteObjectsRequest Request;
typedef simRosCopyPasteObjectsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosCopyPasteObjects
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "185cb01118006e816646e4234283fa15";
  }

  static const char* value(const  ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x185cb01118006e81ULL;
  static const uint64_t static_value2 = 0x6646e4234283fa15ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosCopyPasteObjectsRequest";
  }

  static const char* value(const  ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
int32[] objectHandles\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "33b2dafaeb9036bc8402196ac964ff11";
  }

  static const char* value(const  ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x33b2dafaeb9036bcULL;
  static const uint64_t static_value2 = 0x8402196ac964ff11ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosCopyPasteObjectsResponse";
  }

  static const char* value(const  ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
int32[] newObjectHandles\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.objectHandles);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosCopyPasteObjectsRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
    stream.next(m.newObjectHandles);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosCopyPasteObjectsResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosCopyPasteObjects> {
  static const char* value() 
  {
    return "6c736eb743dc1dca49bfc35e3f009776";
  }

  static const char* value(const vrep_common::simRosCopyPasteObjects&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosCopyPasteObjects> {
  static const char* value() 
  {
    return "vrep_common/simRosCopyPasteObjects";
  }

  static const char* value(const vrep_common::simRosCopyPasteObjects&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6c736eb743dc1dca49bfc35e3f009776";
  }

  static const char* value(const vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosCopyPasteObjects";
  }

  static const char* value(const vrep_common::simRosCopyPasteObjectsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6c736eb743dc1dca49bfc35e3f009776";
  }

  static const char* value(const vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosCopyPasteObjects";
  }

  static const char* value(const vrep_common::simRosCopyPasteObjectsResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSCOPYPASTEOBJECTS_H

