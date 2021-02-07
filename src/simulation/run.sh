#!/bin/bash
BASEDIR=$(dirname $0)

function usage() {
cat << EOF
    Usage:
    $0 <options ...>
        -g <show gui> (default: false)
EOF
}

# Default docker-compose
COMPOSE=$BASEDIR/gazebo/docker-compose.yml

for i in "$@"
do
    case $i in
        -g)
        echo "Running Gazebo container with GUI"
        COMPOSE=$BASEDIR/gazebo/docker-compose-gui.yml
        ;;
        --help*)
        usage; exit 0;
        ;;
        *)
            # unknown option
        ;;
    esac
done



docker-compose -f $COMPOSE up