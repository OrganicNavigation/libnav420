cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libnav420
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Driver library for the Crossbow NAV420 IMU."
  AUTHOR "Pierre Lamon"
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/libnav420"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX nav420-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  DEPENDS libelrob
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS libnav420 libelrob-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libelrob-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBNAV420_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libelrob-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBNAV420_GIT_REVISION
)
