function(navigation2d_configure_mpc_backend target)
  if(NAVIGATION2D_ACADOS_GENERATED_DIR)
    if(NOT NAVIGATION2D_ACADOS_ROOT)
      message(FATAL_ERROR "NAVIGATION2D_ACADOS_ROOT is required with NAVIGATION2D_ACADOS_GENERATED_DIR")
    endif()
    set(NAVIGATION2D_ACADOS_SOLVER
        "${NAVIGATION2D_ACADOS_GENERATED_DIR}/libacados_ocp_solver_navigation2d_mpcc.so")
    if(NOT EXISTS "${NAVIGATION2D_ACADOS_SOLVER}")
      message(FATAL_ERROR "generated acados solver not found: ${NAVIGATION2D_ACADOS_SOLVER}")
    endif()
    target_sources(${target} PRIVATE control/acados_mpc_backend.cc)
    target_include_directories(${target} PRIVATE
        "${NAVIGATION2D_ACADOS_GENERATED_DIR}" "${NAVIGATION2D_ACADOS_ROOT}/include"
        "${NAVIGATION2D_ACADOS_ROOT}/include/blasfeo/include"
        "${NAVIGATION2D_ACADOS_ROOT}/include/hpipm/include")
    target_link_libraries(${target} PRIVATE "${NAVIGATION2D_ACADOS_SOLVER}"
        "${NAVIGATION2D_ACADOS_ROOT}/lib/libacados.so"
        "${NAVIGATION2D_ACADOS_ROOT}/lib/libhpipm.so"
        "${NAVIGATION2D_ACADOS_ROOT}/lib/libblasfeo.so")
  else()
    target_sources(${target} PRIVATE control/acados_mpc_backend_stub.cc)
  endif()
endfunction()
