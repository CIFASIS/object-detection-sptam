# CMake generated Testfile for 
# Source directory: /home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/tests
# Build directory: /home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ApproxMVBBTests-ConvexHull "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/tests/ApproxMVBBTests-ConvexHull")
add_test(ApproxMVBBTests-MinAreaRect "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/tests/ApproxMVBBTests-MinAreaRect")
add_test(ApproxMVBBTests-Diameter "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/tests/ApproxMVBBTests-Diameter")
add_test(ApproxMVBBTests-DiameterOOBB "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/tests/ApproxMVBBTests-DiameterOOBB")
add_test(ApproxMVBBTests-MVBB "/home/taihu/Documentos/jcorti_bkp/ApproxMVBB2/ApproxMVBB/tests/ApproxMVBBTests-MVBB")
subdirs(../thirdparty/googletest-build)
