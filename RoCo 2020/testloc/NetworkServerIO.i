/*interface file*/

%module NetworkServerIO
 %{
 #define SWIG_FILE_WITH_INIT
 #include "NetworkServerIO.h"
 #include "Build.h"
 #include "IODriver.h"
 #include "NetworkIO.h"
 %}

 %include "NetworkServerIO.h" #include "NetworkServerIO.cpp"
 %include "NetworkServerIO.cpp"
 %include "Build.h"
 %include "IODriver.h"
 %include "NetworkIO.h"
