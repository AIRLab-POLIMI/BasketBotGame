/* Auto-generated by genmsg_cpp for file /home/matteo/catkin_ws/src/vrep/vrep_common/srv/simRosGetInfo.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETINFO_H
#define VREP_COMMON_SERVICE_SIMROSGETINFO_H
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



#include "std_msgs/Header.h"

namespace vrep_common
{
template <class ContainerAllocator>
struct simRosGetInfoRequest_ {
  typedef simRosGetInfoRequest_<ContainerAllocator> Type;

  simRosGetInfoRequest_()
  {
  }

  simRosGetInfoRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetInfoRequest
typedef  ::vrep_common::simRosGetInfoRequest_<std::allocator<void> > simRosGetInfoRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetInfoRequest> simRosGetInfoRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetInfoRequest const> simRosGetInfoRequestConstPtr;



template <class ContainerAllocator>
struct simRosGetInfoResponse_ {
  typedef simRosGetInfoResponse_<ContainerAllocator> Type;

  simRosGetInfoResponse_()
  : headerInfo()
  , simulatorState(0)
  , simulationTime(0.0)
  , timeStep(0.0)
  {
  }

  simRosGetInfoResponse_(const ContainerAllocator& _alloc)
  : headerInfo(_alloc)
  , simulatorState(0)
  , simulationTime(0.0)
  , timeStep(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _headerInfo_type;
   ::std_msgs::Header_<ContainerAllocator>  headerInfo;

  typedef int32_t _simulatorState_type;
  int32_t simulatorState;

  typedef float _simulationTime_type;
  float simulationTime;

  typedef float _timeStep_type;
  float timeStep;


  typedef boost::shared_ptr< ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetInfoResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetInfoResponse
typedef  ::vrep_common::simRosGetInfoResponse_<std::allocator<void> > simRosGetInfoResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetInfoResponse> simRosGetInfoResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetInfoResponse const> simRosGetInfoResponseConstPtr;


struct simRosGetInfo
{

typedef simRosGetInfoRequest Request;
typedef simRosGetInfoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetInfo
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetInfoRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetInfoResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2ab24cc264f8f17af7e013147c57dbc0";
  }

  static const char* value(const  ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2ab24cc264f8f17aULL;
  static const uint64_t static_value2 = 0xf7e013147c57dbc0ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetInfoResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header headerInfo\n\
int32 simulatorState\n\
float32 simulationTime\n\
float32 timeStep\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetInfoRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetInfoRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetInfoResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.headerInfo);
    stream.next(m.simulatorState);
    stream.next(m.simulationTime);
    stream.next(m.timeStep);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetInfoResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetInfo> {
  static const char* value() 
  {
    return "2ab24cc264f8f17af7e013147c57dbc0";
  }

  static const char* value(const vrep_common::simRosGetInfo&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetInfo> {
  static const char* value() 
  {
    return "vrep_common/simRosGetInfo";
  }

  static const char* value(const vrep_common::simRosGetInfo&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetInfoRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2ab24cc264f8f17af7e013147c57dbc0";
  }

  static const char* value(const vrep_common::simRosGetInfoRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetInfoRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetInfo";
  }

  static const char* value(const vrep_common::simRosGetInfoRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetInfoResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2ab24cc264f8f17af7e013147c57dbc0";
  }

  static const char* value(const vrep_common::simRosGetInfoResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetInfoResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetInfo";
  }

  static const char* value(const vrep_common::simRosGetInfoResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETINFO_H

