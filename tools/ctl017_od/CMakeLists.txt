# SPDX-License-Identifier: MIT
add_executable(ctl017_od_dump od_dump.c)
target_link_libraries(ctl017_od_dump PRIVATE ctl017_app)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  # Fails if docs/ctl017.eds or the generated table in
  # docs/ctl017_object_dictionary.md differ from the firmware OD.
  add_test(NAME od_docs_up_to_date
    COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/gen_od_docs.py
            --dump $<TARGET_FILE:ctl017_od_dump> --docs ${PROJECT_SOURCE_DIR}/docs --check)
endif()
