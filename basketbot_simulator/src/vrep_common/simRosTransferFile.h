/* Auto-generated by genmsg_cpp for file /home/matteo/catkin_ws/src/vrep/vrep_common/srv/simRosTransferFile.srv */
#ifndef VREP_COMMON_SERVICE_SIMROSTRANSFERFILE_H
#define VREP_COMMON_SERVICE_SIMROSTRANSFERFILE_H
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
struct simRosTransferFileRequest_ {
  typedef simRosTransferFileRequest_<ContainerAllocator> Type;

  simRosTransferFileRequest_()
  : data()
  , fileName()
  {
  }

  simRosTransferFileRequest_(const ContainerAllocator& _alloc)
  : data(_alloc)
  , fileName(_alloc)
  {
  }

  typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  data;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _fileName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  fileName;


  typedef boost::shared_ptr< ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosTransferFileRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosTransferFileRequest
typedef  ::vrep_common::simRosTransferFileRequest_<std::allocator<void> > simRosTransferFileRequest;

typedef boost::shared_ptr< ::vrep_common::simRosTransferFileRequest> simRosTransferFileRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosTransferFileRequest const> simRosTransferFileRequestConstPtr;



template <class ContainerAllocator>
struct simRosTransferFileResponse_ {
  typedef simRosTransferFileResponse_<ContainerAllocator> Type;

  simRosTransferFileResponse_()
  : result(0)
  {
  }

  simRosTransferFileResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int32_t _result_type;
  int32_t result;


  typedef boost::shared_ptr< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct simRosTransferFileResponse
typedef  ::vrep_common::simRosTransferFileResponse_<std::allocator<void> > simRosTransferFileResponse;

typedef boost::shared_ptr< ::vrep_common::simRosTransferFileResponse> simRosTransferFileResponsePtr;
typedef boost::shared_ptr< ::vrep_common::simRosTransferFileResponse const> simRosTransferFileResponseConstPtr;


struct simRosTransferFile
{

typedef simRosTransferFileRequest Request;
typedef simRosTransferFileResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct simRosTransferFile
} // namespace vrep_common

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosTransferFileRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c8b761e9b02a5ad1aa85aa2c994c5e52";
  }

  static const char* value(const  ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc8b761e9b02a5ad1ULL;
  static const uint64_t static_value2 = 0xaa85aa2c994c5e52ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosTransferFileRequest";
  }

  static const char* value(const  ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
uint8[] data\n\
string fileName\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const  ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosTransferFileResponse";
  }

  static const char* value(const  ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosTransferFileRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
    stream.next(m.fileName);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosTransferFileRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosTransferFileResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct simRosTransferFileResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vrep_common::simRosTransferFile> {
  static const char* value() 
  {
    return "3c956e5a32cec1f93100fec7ced2ccd4";
  }

  static const char* value(const vrep_common::simRosTransferFile&) { return value(); } 
};

template<>
struct DataType<vrep_common::simRosTransferFile> {
  static const char* value() 
  {
    return "vrep_common/simRosTransferFile";
  }

  static const char* value(const vrep_common::simRosTransferFile&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosTransferFileRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3c956e5a32cec1f93100fec7ced2ccd4";
  }

  static const char* value(const vrep_common::simRosTransferFileRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosTransferFileRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosTransferFile";
  }

  static const char* value(const vrep_common::simRosTransferFileRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vrep_common::simRosTransferFileResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3c956e5a32cec1f93100fec7ced2ccd4";
  }

  static const char* value(const vrep_common::simRosTransferFileResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vrep_common::simRosTransferFileResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vrep_common/simRosTransferFile";
  }

  static const char* value(const vrep_common::simRosTransferFileResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_SERVICE_SIMROSTRANSFERFILE_H

