//
// Created by stefan on 23.01.21.
//

#include "ros2_babel_fish/idl/serialization.hpp"
#include "ros2_babel_fish/macros.hpp"
#include "ros2_babel_fish/messages/compound_message.hpp"
#include "ros2_babel_fish/messages/value_message.hpp"

namespace ros2_babel_fish
{
std::shared_ptr<void>
createContainer( const rosidl_typesupport_introspection_cpp::MessageMembers &members,
                 rosidl_runtime_cpp::MessageInitialization initialization )
{
  auto result = std::shared_ptr<void>( new unsigned char[members.size_of_], [members]( void *data ) {
    members.fini_function( data );
    delete[] static_cast<unsigned char *>( data );
  } );
  members.init_function( result.get(), initialization );
  return result;
}

std::shared_ptr<void> createContainer( const MessageMembersIntrospection &members,
                                       rosidl_runtime_cpp::MessageInitialization initialization )
{
  return createContainer( *members.value, initialization );
}
} // namespace ros2_babel_fish
