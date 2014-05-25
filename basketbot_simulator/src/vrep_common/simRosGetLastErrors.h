/* Auto-generated by genmsg_cpp for file /home/matteo/catkin_ws/src/vrep/vrep_common/srv/simRosGetLastErrors.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSGETLASTERRORS_H
#define VREP_COMMON_SERVICE_SIMROSGETLASTERRORS_H
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
struct simRosGetLastErrorsRequest_ {
  typedef simRosGetLastErrorsRequest_<ContainerAllocator> Type;

  simRosGetLastErrorsRequest_()
  {
  }

  simRosGetLastErrorsRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetLastErrorsRequest
typedef  ::vrep_common::simRosGetLastErrorsRequest_<std::allocator<void> > simRosGetLastErrorsRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetLastErrorsRequest> simRosGetLastErrorsRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetLastErrorsRequest const> simRosGetLastErrorsRequestConstPtr;



template <class ContainerAllocator>
struct simRosGetLastErrorsResponse_ {
  typedef simRosGetLastErrorsResponse_<ContainerAllocator> Type;

  simRosGetLastErrorsResponse_()
  : errorCnt(0)
  , errorStrings()
  {
  }

  simRosGetLastErrorsResponse_(const ContainerAllocator& _alloc)
  : errorCnt(0)
  , errorStrings(_alloc)
  {
  }

  typedef int32_t _errorCnt_type;
  int32_t errorCnt;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _errorStrings_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  errorStrings;


  typedef boost::shared_ptr< ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosGetLastErrorsResponse
typedef  ::vrep_common::simRosGetLastErrorsResponse_<std::allocator<void> > simRosGetLastErrorsResponse;

typedef boost::shared_ptr< ::vrep_common::simRosGetLastErrorsResponse> simRosGetLastErrorsResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetLastErrorsResponse const> simRosGetLastErrorsResponseConstPtr;


struct simRosGetLastErrors
{

typedef simRosGetLastErrorsRequest Request;
typedef simRosGetLastErrorsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosGetLastErrors
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetLastErrorsRequest";
  }

  static const char* value(const  ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> > {
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

  static const char* value(const  ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "165502f1a9cdf7b50941ef103382dea5";
  }

  static const char* value(const  ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x165502f1a9cdf7b5ULL;
  static const uint64_t static_value2 = 0x0941ef103382dea5ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetLastErrorsResponse";
  }

  static const char* value(const  ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 errorCnt\n\
string errorStrings\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetLastErrorsRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.errorCnt);
    stream.next(m.errorStrings);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosGetLastErrorsResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosGetLastErrors> {
  static const char* value() 
  {
    return "165502f1a9cdf7b50941ef103382dea5";
  }

  static const char* value(const vrep_common::simRosGetLastErrors&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosGetLastErrors> {
  static const char* value() 
  {
    return "vrep_common/simRosGetLastErrors";
  }

  static const char* value(const vrep_common::simRosGetLastErrors&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "165502f1a9cdf7b50941ef103382dea5";
  }

  static const char* value(const vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetLastErrors";
  }

  static const char* value(const vrep_common::simRosGetLastErrorsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "165502f1a9cdf7b50941ef103382dea5";
  }

  static const char* value(const vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosGetLastErrors";
  }

  static const char* value(const vrep_common::simRosGetLastErrorsResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSGETLASTERRORS_H

