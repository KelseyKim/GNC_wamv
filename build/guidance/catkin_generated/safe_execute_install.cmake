execute_process(COMMAND "/home/ktkim/Workspaces/GNC_wamv/build/guidance/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ktkim/Workspaces/GNC_wamv/build/guidance/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
