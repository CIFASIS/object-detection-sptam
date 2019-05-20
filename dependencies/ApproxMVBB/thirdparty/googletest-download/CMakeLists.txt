cmake_minimum_required(VERSION 2.8.2)

project(googletest-download NONE)


include(ExternalProject)
ExternalProject_Add(googletest-download
                    GIT_REPOSITORY;https://github.com/google/googletest.git;GIT_TAG;master;UPDATE_DISCONNECTED 1
                    SOURCE_DIR          "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/thirdparty/googletest-src"
                    BINARY_DIR          "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/thirdparty/googletest-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
