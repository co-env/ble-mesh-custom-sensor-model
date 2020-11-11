# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# set(srcs "src/mesh_client.c" 
#         # "src/mesh_server.c")
#     )

# idf_component_register(SRCS "${src}"
#                     INCLUDE_DIRS "include"
#                     REQUIRES bt)

set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_SRCS "src/mesh_client.c"
                    "src/mesh_server.c"
                    "src/mesh_device_app.c")

set(COMPONENT_REQUIRES "bt")

register_component()