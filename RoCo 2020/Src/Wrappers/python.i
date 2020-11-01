%module RoCo2
%{
#define BUILD_FOR_CONTROL_STATION
#define PROTOCOL_20W18

#include "../Build/Build.h"
#include "../Protocol/Protocol.h"

#include "../NetworkClientIO.h"
#include "../IOBus.h"
#include "../MessageBus.h"
#include "../NetworkBus.h"
%}

%define ASINT(T)
	%typemap(in) T {
	    $1 = PyInt_AsLong($input);
	}

	%typemap(out) T {
	    $result = PyInt_FromLong($1);
	}
%enddef

%define REGISTER(T)
	%template(define ## T) NetworkBus::define<T>;
	%template(handle ## T) NetworkBus::handle<T>;
	%template(forward ## T) NetworkBus::forward<T>;
	%template(send ## T) NetworkBus::send<T>;
%enddef


ASINT(uint8_t);
ASINT(uint16_t);
ASINT(uint32_t);
ASINT(uint64_t);
ASINT(int8_t);
ASINT(int16_t);
ASINT(int32_t);
ASINT(int64_t);
ASINT(bool);

#define __attribute__(x)

#define BUILD_FOR_CONTROL_STATION
#define PROTOCOL_20W18

%include "../Build/Build.h"

%include "../Protocol/Protocol20W18.h"
%include "../IODriver.h"
%include "../NetworkClientIO.h"
%include "../MessageBus.h"
%include "../IOBus.h"
%include "../NetworkBus.h"
%include "../Protocol/ProtocolRegisters.h"
