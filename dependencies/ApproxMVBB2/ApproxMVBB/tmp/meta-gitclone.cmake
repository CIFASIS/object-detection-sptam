if("master" STREQUAL "")
  message(FATAL_ERROR "Tag for git checkout should not be empty.")
endif()

set(run 0)

if("/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta-stamp/meta-gitinfo.txt" IS_NEWER_THAN "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta-stamp/meta-gitclone-lastrun.txt")
  set(run 1)
endif()

if(NOT run)
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta-stamp/meta-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta'")
endif()

# try the clone 3 times incase there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git" clone --origin "origin" "https://github.com/ericniebler/meta.git" "meta"
    WORKING_DIRECTORY "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/ericniebler/meta.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git" checkout master
  WORKING_DIRECTORY "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'master'")
endif()

execute_process(
  COMMAND "/usr/bin/git" submodule init 
  WORKING_DIRECTORY "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to init submodules in: '/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta'")
endif()

execute_process(
  COMMAND "/usr/bin/git" submodule update --recursive 
  WORKING_DIRECTORY "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta-stamp/meta-gitinfo.txt"
    "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta-stamp/meta-gitclone-lastrun.txt"
  WORKING_DIRECTORY "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/src/meta-stamp/meta-gitclone-lastrun.txt'")
endif()

