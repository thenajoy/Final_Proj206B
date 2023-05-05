// Generated by gencpp from file qrotor_firmware/Setpoint.msg
// DO NOT EDIT!


#ifndef QROTOR_FIRMWARE_MESSAGE_SETPOINT_H
#define QROTOR_FIRMWARE_MESSAGE_SETPOINT_H

#include <ros/service_traits.h>


#include <qrotor_firmware/SetpointRequest.h>
#include <qrotor_firmware/SetpointResponse.h>


namespace qrotor_firmware
{

struct Setpoint
{

typedef SetpointRequest Request;
typedef SetpointResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Setpoint
} // namespace qrotor_firmware


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::qrotor_firmware::Setpoint > {
  static const char* value()
  {
    return "14fb54e9e518f55d418823395ca25d0b";
  }

  static const char* value(const ::qrotor_firmware::Setpoint&) { return value(); }
};

template<>
struct DataType< ::qrotor_firmware::Setpoint > {
  static const char* value()
  {
    return "qrotor_firmware/Setpoint";
  }

  static const char* value(const ::qrotor_firmware::Setpoint&) { return value(); }
};


// service_traits::MD5Sum< ::qrotor_firmware::SetpointRequest> should match
// service_traits::MD5Sum< ::qrotor_firmware::Setpoint >
template<>
struct MD5Sum< ::qrotor_firmware::SetpointRequest>
{
  static const char* value()
  {
    return MD5Sum< ::qrotor_firmware::Setpoint >::value();
  }
  static const char* value(const ::qrotor_firmware::SetpointRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::qrotor_firmware::SetpointRequest> should match
// service_traits::DataType< ::qrotor_firmware::Setpoint >
template<>
struct DataType< ::qrotor_firmware::SetpointRequest>
{
  static const char* value()
  {
    return DataType< ::qrotor_firmware::Setpoint >::value();
  }
  static const char* value(const ::qrotor_firmware::SetpointRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::qrotor_firmware::SetpointResponse> should match
// service_traits::MD5Sum< ::qrotor_firmware::Setpoint >
template<>
struct MD5Sum< ::qrotor_firmware::SetpointResponse>
{
  static const char* value()
  {
    return MD5Sum< ::qrotor_firmware::Setpoint >::value();
  }
  static const char* value(const ::qrotor_firmware::SetpointResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::qrotor_firmware::SetpointResponse> should match
// service_traits::DataType< ::qrotor_firmware::Setpoint >
template<>
struct DataType< ::qrotor_firmware::SetpointResponse>
{
  static const char* value()
  {
    return DataType< ::qrotor_firmware::Setpoint >::value();
  }
  static const char* value(const ::qrotor_firmware::SetpointResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QROTOR_FIRMWARE_MESSAGE_SETPOINT_H
