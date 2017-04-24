#!/bin/bash
export CLASSPATH=.
## Colorize the ls output ##
alias ls='ls --color=auto'

## Use a long listing format ##
alias ll='ls -la'

## Show hidden files ##
alias l.='ls -d .* --color=auto'

## Initialize & validate sedona dev (unix) environment
source ~/sedonadev/adm/unix/init.sh

## Set command prompt
PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
