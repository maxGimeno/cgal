#!/bin/bash

CHECK=
case $1 in
  --check) CHECK=y;;
esac

set -e
cd ../

if [ -f "$PWD/.travis/packages.txt" ] 
then
	rm "$PWD/.travis/packages.txt"
fi

#find all the packages
PACKAGES=()
INDEX=0
i=0
for f in *
do
  if [ -d  "$f/package_info/$f" ]
	then
		PACKAGES[$INDEX]+="$f "
		i=$[i+1]
		if [ $i = 3 ]
		then
			i=0
 			INDEX=$[INDEX+1]
		fi
  	echo "$f " >> ./.travis/packages.txt
	fi
done
if [ -f ".travis.yml" ] 
then
  #copy the current .travis.yml for later check
  mv ./.travis.yml ./.travis.old
fi
#writes the first part of the file	
old_IFS=$IFS       
IFS=$'\n'
for LINE in $(cat "$PWD/.travis/template.txt")
do
	if [ "$LINE" != " include:" ]
	then
		echo "$LINE" >> .travis.yml
  else
  	break
	fi
done
echo " include: " >> .travis.yml
#writes the matrix
echo "  - compiler: gcc " >> .travis.yml
echo "    env: PACKAGE='CHECK' " >> .travis.yml
echo "  - compiler: clang-3.6" >> .travis.yml
echo "    env: PACKAGE='CHECK' " >> .travis.yml
for package in ${PACKAGES[@]}
do
echo "  - compiler: clang-3.6" >> .travis.yml
echo "    env: PACKAGE='$package' " >> .travis.yml
done
echo "  - compiler: clang-3.6" >> .travis.yml
echo "    env: PACKAGE='Polyhedron_demo' " >> .travis.yml

#writes the end of the file
COPY=0
for LINE in $(cat "$PWD/.travis/template.txt")
do
	if [ "$LINE" = "install: " ]
	then
		COPY=1
	fi
	if [ $COPY = 1 ]
	then
		echo "$LINE" >> .travis.yml
	fi
done
IFS=$' '
#check if there are differences between the files
if ! cmp -s ./.travis.yml ./.travis.old;
then
    echo ".travis.yml has changed"
    if [ -n "$CHECK" ]; then
        echo "You should modify the file .travis/template.txt"
        exit 1
    fi
fi
#erase old travis
rm ./.travis.old
IFS=$old_IFS

# Local Variables:
# tab-width: 2
# sh-basic-offset: 2
# End:
