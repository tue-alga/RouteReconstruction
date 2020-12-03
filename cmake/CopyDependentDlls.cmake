# Directory of this file. Needed to resolve the python file
set(_CopyDependentDlls_DIR ${CMAKE_CURRENT_LIST_DIR})

function(CopyDependentDlls TargetName)
    # This does a post build 
    include(FindPythonInterp)

    set(_BIN_NAME ${TargetName})
    if(${ARGC} EQUAL 2)
        set(_BIN_NAME ${ARGV1})
    endif()

    get_target_property(_CWD ${TargetName} BINARY_DIR)
    # get_target_property(_OUTNAME ${TargetName} OUTPUT_NAME)
    # if(NOT _OUTNAME)
    #     set(_PF CMAKE_$<UPPER_CASE:$<CONFIG>>_POSTFIX)
    #     set(_PF ${_PF})
    #     set(_OUTNAME ${TargetName}${_PF}>)
        
    #     message(STATUS ${_PF})
    #     message(STATUS ${RUNTIME_OUTPUT_NAME})
    # endif()


    add_custom_target(
        DllCopyTarget_${TargetName}
        COMMAND ${PYTHON_EXECUTABLE} ${_CopyDependentDlls_DIR}/CopyDependentDllsScript.py ${TargetName} $<CONFIG> ${_BIN_NAME}
        WORKING_DIRECTORY ${_CWD}
    )
    add_dependencies(DllCopyTarget_${TargetName} ${TargetName})

    #add_custom_target(DllCopyAndRun_${TargetName}
    #    COMMAND ${TargetName}
    #    WORKING_DIRECTORY ${_CWD}
    #)
    #add_dependencies(DllCopyAndRun_${TargetName} DllCopyTarget_${TargetName})
endfunction()