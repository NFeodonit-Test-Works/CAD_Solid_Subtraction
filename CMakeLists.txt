# ****************************************************************************
#  Project:  CAD_TestWork_Subtract_KinematicSolid_from_Parallelepiped
#  Purpose:  Test project
#  Author:   NikitaFeodonit, nfeodonit@yandex.com
# ****************************************************************************
#    Copyright (c) 2019 NikitaFeodonit
#
#    This file is part of the
#    CAD_TestWork_Subtract_KinematicSolid_from_Parallelepiped project.
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published
#    by the Free Software Foundation, either version 3 of the License,
#    or (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#    See the GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program. If not, see <http://www.gnu.org/licenses/>.
# ****************************************************************************

cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

project(Test_Moduleworks CXX)
set(tcs_EXE_NAME "${PROJECT_NAME}")

add_executable(${tcs_EXE_NAME} "")
set_target_properties(${tcs_EXE_NAME} PROPERTIES
  CXX_STANDARD 11
)

target_include_directories(${tcs_EXE_NAME} PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/src"
)

target_sources(${tcs_EXE_NAME} PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/src/TaskSolution.cpp"

  "${CMAKE_CURRENT_LIST_DIR}/src/CreateSkin.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/cutSphereMove.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwArcFunction.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwException.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwIdxRangeException.cpp"

  "${CMAKE_CURRENT_LIST_DIR}/src/TaskSolution.hpp"

  "${CMAKE_CURRENT_LIST_DIR}/src/CreateSkin.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwArcFunction.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwDiscreteFunction.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwException.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwIdxRangeException.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwMathConstants.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/src/mwTPoint3d.hpp"
)
