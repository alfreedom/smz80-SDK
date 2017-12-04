#!/bin/sh

# Copyright 2010 One Laptop per Child
# License information below.

# produce a listing of the code and initialized constant sizes
# for the all the obj/*.rel files produced by sdcc.
# requires dc, xargs, cut, sed.

me=${0##*/}

tmpfile="/tmp/$me.$$"
trap "rm -f $tmpfile" 0

total_size=0

main()
{
    
    listsizes | tee $tmpfile

    echo
    echo " "
    echo "   Memoria           Tamaño (Bytes)"
    echo "-----------------------------------"
    echo -n "PROGRAMA (EEPROM):       "
    sumall 2 0 4 5 6 0 < $tmpfile
    echo -n "DATOS  (RAM):            "
    sumall 0 3 0 0 0 7 < $tmpfile
    echo "-----------------------------------"
    echo -n "TAMAÑO TOTAL:            " 
    sumall 2 3 3 5 6 7 < $tmpfile
    echo "\n"
}


listsizes()
{
    for x in _build/*.rel
     do
	  (
	    echo :
	    (
	    egrep 'A _CODE' $x;
        egrep 'A _DATA' $x;
        egrep 'A _HOME' $x;
        egrep 'A _GSINIT' $x;
	    egrep 'A _GSFINAL' $x;
	    egrep 'A _INITIALIZED' $x;
	    )|
		sed -e 's/A //' \
		    -e 's/size //' \
		    -e 's/ flags.*//' \
		    -e 's/_CODE\|_DATA\|_HOME\|_GSINIT\|_GSFINAL\|_INITIALIZED//' \
		    -e 's/[0-9A-F]\+/0x&/g'
	   ) | 
	   xargs -n 7 printf "%s %7d %7d %7d %7d %7d %7d\n" 
    done
}

# tally the Nth field of the input
sumcol()
{
    awk '
	BEGIN	{ total = 0; }
		{ total += $'$1'  
        }
	END	{ 
            print total ; 
        }
	'
}

# tally the Nth field of the input
sumall()
{
    awk '
    BEGIN   { total = 0; }
        { total += $'$1'  
          total += $'$2'  
          total += $'$3'  
          total += $'$4'  
          total += $'$5'
          total += $'$6'
        }
    END { 
            print total ""; 
        }
    '
}

main

exit