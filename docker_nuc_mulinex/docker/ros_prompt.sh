#!/bin/sh
# Show user, ROS_DOMAIN_ID and RMW implementation in prompt for interactive shells
case "$-" in
  *i*)
    __ros_update_ps1() {
      PS1="\[\e[32m\]\u\[\e[0m\] | \[\e[36m\]ROS_ID:${ROS_DOMAIN_ID:-}\[\e[0m\] | \[\e[33m\]DDS:${RMW_IMPLEMENTATION#rmw_}\[\e[0m\] | \[\e[34m\]\h\[\e[0m\] \[\e[34m\]\W\[\e[0m\] \$ "
    }
    export PROMPT_COMMAND=__ros_update_ps1
    ;;
esac
