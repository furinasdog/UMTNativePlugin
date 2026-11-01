# SPDX-License-Identifier: MIT
# Only the dependency build cache is written; never reset or clean an existing checkout.
if(NOT BULLET_SOURCE_DIR OR NOT BULLET_COMMIT OR NOT GIT_EXECUTABLE)
    message(FATAL_ERROR "Missing pinned Bullet source, commit or Git executable.")
endif()

function(run_git)
    execute_process(COMMAND "${GIT_EXECUTABLE}" ${ARGN}
        RESULT_VARIABLE _status OUTPUT_VARIABLE _out ERROR_VARIABLE _error)
    if(NOT _status EQUAL 0)
        message(FATAL_ERROR "Bullet Git preparation failed: ${_error}")
    endif()
endfunction()

if(NOT EXISTS "${BULLET_SOURCE_DIR}/.git")
    if(NOT BULLET_FETCH_ALLOWED)
        message(FATAL_ERROR "BulletRoot must be a Git checkout of the pinned commit.")
    endif()
    if(EXISTS "${BULLET_SOURCE_DIR}/src")
        message(FATAL_ERROR "Unmanaged Bullet source already exists; preserve it and select a different build cache.")
    endif()
    # Partial/sparse checkout excludes the repository's large example assets.
    run_git(-c core.autocrlf=false clone --filter=blob:none --no-checkout --depth 1 --single-branch
        https://github.com/bulletphysics/bullet3.git "${BULLET_SOURCE_DIR}")
    run_git(-C "${BULLET_SOURCE_DIR}" config core.autocrlf false)
    run_git(-C "${BULLET_SOURCE_DIR}" fetch --filter=blob:none --depth 1 origin "${BULLET_COMMIT}")
    run_git(-C "${BULLET_SOURCE_DIR}" sparse-checkout init --cone)
    run_git(-C "${BULLET_SOURCE_DIR}" checkout --detach "${BULLET_COMMIT}")
    run_git(-C "${BULLET_SOURCE_DIR}" sparse-checkout set src)
endif()

execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BULLET_SOURCE_DIR}" rev-parse HEAD
    RESULT_VARIABLE _status OUTPUT_VARIABLE _head OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _status EQUAL 0 OR NOT _head STREQUAL BULLET_COMMIT)
    message(FATAL_ERROR "BulletRoot does not match pinned commit ${BULLET_COMMIT}; refusing stale/other Bullet sources.")
endif()
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BULLET_SOURCE_DIR}" status --porcelain --untracked-files=all -- src
    RESULT_VARIABLE _status OUTPUT_VARIABLE _dirty OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT _status EQUAL 0 OR _dirty)
    message(FATAL_ERROR "Bullet source has local changes; refusing an unverified dependency. No files were reset.")
endif()
if(NOT EXISTS "${BULLET_SOURCE_DIR}/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp")
    message(FATAL_ERROR "Pinned Bullet src is not present; initialize the sparse checkout first.")
endif()
file(SHA256 "${BULLET_SOURCE_DIR}/LICENSE.txt" _license)
if(NOT _license STREQUAL "885cd516a8420acb7c3e0b1a7119913b686495001ae564e3229740460e8cff59")
    message(FATAL_ERROR "Pinned Bullet license checksum mismatch.")
endif()
