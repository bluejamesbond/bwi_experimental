#!/bin/bash
# tputcolors

# text color variables
txtund=$(tput sgr 0 1)          # Underline
txtbld=$(tput bold)             # Bold
bldred=${txtbld}$(tput setaf 1) #  red
bldblu=${txtbld}$(tput setaf 4) #  blue
bldgre=${txtbld}$(tput setaf 2) #  green
bldyel=${txtbld}$(tput setaf 3) #  yellow
txtrst=$(tput sgr0)             # Reset

function timeout (){
    for index in $(eval "echo {$1..0}")
    do
        echo "[$CONSOLE_TAG] $bldyel$2$txtrst - $index seconds remaining"
        sleep 1
    done
}

function log (){
    echo "[$CONSOLE_TAG] $bldgre$1$txtrst"
}

function warn (){
    echo "[$CONSOLE_TAG] $bldyel$1$txtrst"
}

function err (){
    echo "[$CONSOLE_TAG] $bldred$1$txtrst"
}

function info (){
    echo "[$CONSOLE_TAG] $bldblu$1$txtrst"
}
