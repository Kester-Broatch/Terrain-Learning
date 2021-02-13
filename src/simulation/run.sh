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
COMPOSE_DIR=$BASEDIR/docker/docker-compose.yml

for i in "$@"
do
    case $i in
        -g)
        echo "Running Gazebo container with GUI"
        COMPOSE_DIR=$BASEDIR/docker/docker-compose-gui.yml
        ;;
        --help*)
        usage; exit 0;
        ;;
        *)
            # unknown option
        ;;
    esac
done

# Start docker containers
docker-compose -f $COMPOSE_DIR up --build

# Clean up 
# echo "Cleaning up"
# docker-compose -f $COMPOSE_DIR down