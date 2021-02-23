#!/bin/sh

#
# Copyright (C) 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Jenkins will pass -xml, in which case we want to generate XML output
xmlout=0
if test "$1" = "-xmldir" -a -n "$2"; then
  xmlout=1
  xmldir=$2
  mkdir -p $xmldir
  rm -rf $xmldir/*.xml
  # Assuming that Jenkins called, the `build` directory is sibling to src dir
  builddir=../build
else
  # This is a heuristic guess; not everyone puts the `build` dir in the src dir
  builddir=./build
fi

# Identify cppcheck version
CPPCHECK_VERSION=`cppcheck --version | sed -e 's@Cppcheck @@'`
CPPCHECK_LT_157=`echo "$CPPCHECK_VERSION 1.57" | \
                 awk '{if ($1 < $2) print 1; else print 0}'`

QUICK_CHECK=0
if test "$1" = "--quick"
then
  QUICK_CHECK=1
  QUICK_SOURCE=$2
  hg_root=`hg root`
  if [ "$?" -ne "0" ] ; then
    echo This is not an hg repository
    exit
  fi
  cd $hg_root
  hg log -r $QUICK_SOURCE > /dev/null
  if [ "$?" -ne "0" ] ; then
    echo $QUICK_SOURCE is not a valid changeset hash
    exit
  fi
  CHECK_FILES=""
  while read line; do
    for f in $line; do
      CHECK_FILES="$CHECK_FILES `echo $f | grep '\.[ch][ch]*$' | grep -v '^deps'`"
    done
  done
  CPPCHECK_FILES="$CHECK_FILES"
  CPPLINT_FILES="$CHECK_FILES"
  QUICK_TMP=`mktemp -t asdfXXXXXXXXXX`
else
  CHECK_DIRS=" \
    ./usv_gazebo_plugins/src ./usv_gazebo_plugins/include \
    ./vrx_gazebo/src ./vrx_gazebo/include \
    ./wave_gazebo_plugins/src ./wave_gazebo_plugins/include \
  "
  if [ $CPPCHECK_LT_157 -eq 1 ]; then
    # cppcheck is older than 1.57, so don't check header files (issue #907)
    CPPCHECK_FILES=`find $CHECK_DIRS -name "*.cc"`
  else
    CPPCHECK_FILES=`find $CHECK_DIRS -name "*.cc" -o -name "*.hh"`
  fi
  CPPLINT_FILES=`\
    find $CHECK_DIRS -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h"\
        | grep -v -e NetUtils -e Helpers -e example/build`
fi

SUPPRESS=/tmp/cpp_check.suppress
touch $SUPPRESS
# The follow suppression is useful when checking for missing includes.
# It's disable for now because checking for missing includes is very
# time consuming. See CPPCHECK_CMD3.
#echo "missingIncludeSystem" >> $SUPPRESS

#cppcheck
CPPCHECK_BASE="cppcheck -q --suppressions-list=$SUPPRESS --inline-suppr"
if [ $CPPCHECK_LT_157 -eq 0 ]; then
  # use --language argument if 1.57 or greater (issue #907)
  CPPCHECK_BASE="$CPPCHECK_BASE --language=c++"
fi
CPPCHECK_INCLUDES="-I . -I $builddir -I test -I include"
CPPCHECK_RULES="-DIGNITION_TRANSPORT_VISIBLE"
CPPCHECK_CMD1A="-j 4 --enable=style,performance,portability"
CPPCHECK_CMD1B="$CPPCHECK_RULES $CPPCHECK_FILES"
CPPCHECK_CMD1="$CPPCHECK_CMD1A $CPPCHECK_CMD1B -I include"
CPPCHECK_CMD2="--enable=unusedFunction $CPPCHECK_FILES"

# Checking for missing includes is very time consuming. This is disabled
# for now
CPPCHECK_CMD3="-j 4 --enable=missingInclude $CPPCHECK_FILES"\
" $CPPCHECK_INCLUDES"
#CPPCHECK_CMD3=""

if [ $xmlout -eq 1 ]; then
  # Performance, style, portability, and information
  ($CPPCHECK_BASE --xml $CPPCHECK_CMD1) 2> $xmldir/cppcheck.xml

  # Check the configuration
  ($CPPCHECK_BASE --xml $CPPCHECK_CMD3) 2> $xmldir/cppcheck-configuration.xml
elif [ $QUICK_CHECK -eq 1 ]; then
  for f in $CHECK_FILES; do
    prefix=`basename $f | sed -e 's@\..*$@@'`
    ext=`echo $f | sed -e 's@^.*\.@@'`
    tmp2="$QUICK_TMP"."$ext"
    tmp2base=`basename "$QUICK_TMP"`
    hg cat -r $QUICK_SOURCE $hg_root/$f > $tmp2

    # Fix suppressions for tmp files
    sed -i -e "s@$f@$tmp2@" $SUPPRESS

    # Skip cppcheck for header files if cppcheck is old
    DO_CPPCHECK=0
    if [ $ext = 'cc' ]; then
      DO_CPPCHECK=1
    elif [ $CPPCHECK_LT_157 -eq 0 ]; then
      DO_CPPCHECK=1
    fi

    if [ $DO_CPPCHECK -eq 1 ]; then
      $CPPCHECK_BASE $CPPCHECK_CMD1A $CPPCHECK_RULES $tmp2 2>&1 \
        | sed -e "s@$tmp2@$f@g" \
        | grep -v 'use --check-config for details' \
        | grep -v 'Include file: .*not found'
    fi

    # Undo changes to suppression file
    sed -i -e "s@$tmp2@$f@" $SUPPRESS

    python $hg_root/tools/cpplint.py $tmp2 2>&1 \
      | sed -e "s@$tmp2@$f@g" -e "s@$tmp2base@$prefix@g" \
      | grep -v 'Total errors found: 0'

    rm $tmp2
  done
  rm $QUICK_TMP
else
  # Performance, style, portability, and information
  $CPPCHECK_BASE $CPPCHECK_CMD1 2>&1

  # Check the configuration
  $CPPCHECK_BASE $CPPCHECK_CMD3 2>&1
fi

# cpplint
if [ $xmlout -eq 1 ]; then
  (echo $CPPLINT_FILES | xargs python tools/cpplint.py 2>&1) \
    | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
elif [ $QUICK_CHECK -eq 0 ]; then
  echo $CPPLINT_FILES | xargs python tools/cpplint.py 2>&1
fi
