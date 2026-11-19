# Play RX's changes to the wifi-sdr and devourer submodules (patches/README.md), applied to their working trees
# at configure time. Idempotent: a patch that is already in is left alone, so configuring again changes nothing.
# A patch that neither applies nor is already in means the submodule pin moved without the patch being
# regenerated; that stops the configure rather than building an adapter driver without the fixes.
find_package(Git REQUIRED)

function(kp_apply_patch dir patch)
  get_filename_component(name ${patch} NAME)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${patch})  # an edited patch re-runs this
  # Two configures at once (Gradle's variants, a desktop and a web build) must not both apply it: hold a lock in
  # the submodule's git dir (outside its working tree, so it shows in no status) while checking and applying.
  execute_process(COMMAND ${GIT_EXECUTABLE} -C ${dir} rev-parse --absolute-git-dir
                  OUTPUT_VARIABLE gitdir OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE grc ERROR_QUIET)
  if(grc EQUAL 0 AND IS_DIRECTORY "${gitdir}")
    file(LOCK "${gitdir}/kepyur-patch.lock" GUARD FUNCTION TIMEOUT 120)
  endif()
  # already applied: the patch reverses cleanly
  execute_process(COMMAND ${GIT_EXECUTABLE} -C ${dir} apply --reverse --check ${patch}
                  RESULT_VARIABLE rev OUTPUT_QUIET ERROR_QUIET)
  if(rev EQUAL 0)
    return()
  endif()
  execute_process(COMMAND ${GIT_EXECUTABLE} -C ${dir} apply --check ${patch}
                  RESULT_VARIABLE fwd OUTPUT_QUIET ERROR_VARIABLE fwd_err)
  if(NOT fwd EQUAL 0)
    execute_process(COMMAND ${GIT_EXECUTABLE} -C ${dir} rev-parse --short HEAD
                    OUTPUT_VARIABLE head OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    message(FATAL_ERROR "patch ${name} neither applies to nor is already in ${dir} (at commit ${head}): the "
                        "submodule pin and the patch are out of step. Regenerate the patch against this commit "
                        "(patches/README.md) or check the submodule out at the commit it was made for.\n${fwd_err}")
  endif()
  execute_process(COMMAND ${GIT_EXECUTABLE} -C ${dir} apply ${patch} RESULT_VARIABLE rc ERROR_VARIABLE err)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "patch ${name}: git apply failed in ${dir}:\n${err}")
  endif()
  message(STATUS "applied ${name} to ${dir}")
endfunction()

# KEPYUR_ROOT: the repository root (native/CMakeLists.txt and web/CMakeLists.txt set it)
kp_apply_patch(${KEPYUR_ROOT}/third_party/wifi-sdr ${KEPYUR_ROOT}/patches/wifi-sdr-0001-playrx-stream.patch)
kp_apply_patch(${KEPYUR_ROOT}/third_party/wifi-sdr/external/devourer
               ${KEPYUR_ROOT}/patches/devourer-0001-playrx-usb.patch)
