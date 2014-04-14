find_program(XENO-CONFIG NAMES xeno-config PATHS $ENV{XENOMAI_PATH}/bin /usr/bin /usr/xenomai/bin)
if (XENO-CONFIG)
   execute_process(COMMAND ${XENO-CONFIG} --xeno-cflags
                   RESULT_VARIABLE XENOMAI_NOTFOUND
                   OUTPUT_VARIABLE RT_DEFS
                   OUTPUT_STRIP_TRAILING_WHITESPACE)
   if (XENOMAI_NOTFOUND)
       message (SEND_WARN "Could not find Xenomai include files (command xeno-config --xeno-cflags failed); building non-RT versions only\n")
   else (XENOMAI_NOTFOUND)
     execute_process(COMMAND ${XENO-CONFIG} --xeno-ldflags
                     RESULT_VARIABLE XENOMAI_NOTFOUND2
		     OUTPUT_VARIABLE RT_LIBS
                     OUTPUT_STRIP_TRAILING_WHITESPACE)
     if (XENOMAI_NOTFOUND2)
       message (SEND_WARN "Could not find Xenomai libs (command xeno-config --xeno-ldflags failed); building non-RT versions only\n")
     else (XENOMAI_NOTFOUND2)
       set (RT_BUILD "true")
     endif (XENOMAI_NOTFOUND2)
   endif(XENOMAI_NOTFOUND)
elseif ("$ENV{XENOMAI_PATH}" STRGREATER "")
   message(STATUS "Using XENOMAI_PATH=$ENV{XENOMAI_PATH}")
   set (RT_DEFS "-I$ENV{XENOMAI_PATH}/include")
   set (RT_LIBS "-L$ENV{XENOMAI_PATH}/lib -lnative")
   set (RT_BUILD "true")
endif (XENO-CONFIG)

if (RT_BUILD)
  set (RT_DEFS "-DOWD_RT ${RT_DEFS}")
  set (RT_LIBS "-lnative ${RT_LIBS}")
endif (RT_BUILD)

